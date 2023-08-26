from queue import Queue
from typing import Dict, List, Set, Tuple, Union

import networkx as nx
from matplotlib.pyplot import Figure, axis, close, figure, pause
from mpl_toolkits.mplot3d import Axes3D  # cspell: disable-line, leave this here, otherwise fig.gca(projection="3d") won't work

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Lambda import NfaLambda
from rt_bi_core.BehaviorAutomaton.SpaceTime import ProjectiveSpaceTimeSet
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.BehaviorAutomaton.TimeRegion import TimeInterval
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.AffineSensorRegion import AffineSensorRegion
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, MultiPolygon, Polygon

Queue.__repr__ = lambda q: repr(q.queue)
___a = Axes3D # This is here to avoid a warning for unused import

class ShadowTree(nx.DiGraph):
	"""The implementation of a Shadow Tree in python as described in the dissertation of Reza Teshnizi."""
	def __init__(self):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__(name="ShadowTree V3")
		self.__fig: Union[Figure, None] = None
		self.__history: List[ConnectivityGraph] = []
		self.componentEvents: List[List[Polygon]] = []
		""" The reason this is a list of lists is that the time of event is relative to the time between. """

		self.redPolys: List[Polygon] = [] # Used for debugging
		self.bluePolys: List[Polygon] = [] # Used for debugging
		self.lines: List[LineString] = [] # Used for debugging

	@property
	def MIN_TIME_DELTA(self) -> float:
		"""
		### A Core Assumption:
		We expect the updates to be at least as fast as 1 ns.
		"""
		return 1

	@property
	def history(self) -> Tuple[ConnectivityGraph, ...]:
		"""Summary description of history."""
		if len(self.__history) == 0:
			return tuple()
		if len(self.__history) == 1:
			return (self.__history[0], )
		if len(self.__history) == 2:
			return (self.__history[0], self.__history[1])
		return (self.__history[0], self.__history[1], self.__history[2])

	def __repr__(self) -> str:
		return self.name

	def __getLowerAndUpperNode(self, n1: str, n2: str) -> Tuple[str, str]:
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def __addNode(self, n: str, timeNanoSecs: int) -> None:
		self.add_node(n)
		self.nodes[n]["fromTime"] = timeNanoSecs
		self.nodes[n]["toTime"] = timeNanoSecs # This will be updated accordingly when adding temporal edges
		return

	def __addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime = None, toTime = None) -> None:
		(lower, upper) = self.__getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)
		return

	def __shadowsAreConnectedTemporally(self, previousGraph: ConnectivityGraph, currentGraph: ConnectivityGraph, previousShadow: dict, currentShadow: dict, centerOfRotation: Geometry.Coords) -> bool:
		"""
			With the assumption that previousNode and currentNode intersect,
			 1. takes the intersection
			 2. finds the polygon made by the translation of each edge
			 3. takes the union of those polygons to get the area swept by FOV
			 4. if the intersection has areas that are not swept by FOV, then they are connected
		"""
		previousP: Polygon = previousShadow["region"].polygon
		currentP: Polygon = currentShadow["region"].polygon
		intersectionOfShadows: Polygon = previousP.intersection(currentP)
		for fov in previousGraph.__sensorNodes:
			if fov not in previousGraph.nodes: continue
			previousFovRegion: SensorRegion = previousGraph.nodes[fov]["region"]
			if fov not in currentGraph.nodes: continue
			currentFovRegion: SensorRegion = currentGraph.nodes[fov]["region"]
			transformation: AffineTransform = Geometry.getAffineTransformation(previousFovRegion.interior, currentFovRegion.interior, centerOfRotation)
			ps = []
			for e in currentFovRegion.__edges:
				edgeC = currentFovRegion.__edges[e]
				edgeP = previousFovRegion.getEquivalentEdge(edgeC, transformation, centerOfRotation)
				if edgeP is None:
					raise AssertionError("No equivalent edge found based on transformation.")
				polygon = Polygon([edgeP.coords[0], edgeP.coords[1], edgeC.coords[1], edgeC.coords[0]])
				ps.append(polygon)
			u = Geometry.union(ps)
			p = intersectionOfShadows.difference(u)
			if p.area > 0.01:
				return True
		return False

	def __checkEdgeIntervalForCollision(self, movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[bool, bool]:
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		movingEdgeStartConfiguration = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		movingEdgeEndConfiguration = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		return (movingEdgeStartConfiguration.intersects(staticEdge), movingEdgeEndConfiguration.intersects(staticEdge))

	def __checkBoundingBoxIntervalForCollision(self, sensor: SensorRegion, movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[Tuple[bool, bool, bool], Polygon, Polygon, LineString]:
		"""
			#### Returns
			Given the configuration information, it returns a tuple: `((bool, bool, bool), Polygon, Polygon)`
			1. A tuple of the state of collision in the interval: `(isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd)`
			2. firstHalfBb
			3. secondHalfBb
		"""
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		edgeAtStart = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtStart = Geometry.intersectLineSegments(edgeAtStart, staticEdge) is not None
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		edgeAtEnd = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtEnd = Geometry.intersectLineSegments(edgeAtEnd, staticEdge) is not None
		intervalMid = (intervalStart + intervalEnd) / 2
		midConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalMid)
		edgeAtMid = Geometry.applyMatrixTransformToLineString(midConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtMid = Geometry.intersectLineSegments(edgeAtMid, staticEdge) is not None
		startSensor = Geometry.applyMatrixTransformToPolygon(startConfigTransformation, sensor.interior, centerOfRotation)
		midSensor = Geometry.applyMatrixTransformToPolygon(midConfigTransformation, sensor.interior, centerOfRotation)
		endSensor = Geometry.applyMatrixTransformToPolygon(endConfigTransformation, sensor.interior, centerOfRotation)
		firstHalfTransformation = Geometry.getAffineTransformation(startSensor, midSensor, centerOfRotation)
		secondHalfTransformation = Geometry.getAffineTransformation(midSensor, endSensor, centerOfRotation)
		firstHalfBb = self._getLineSegmentExpandedBb(firstHalfTransformation, edgeAtStart, firstHalfTransformation.rotation, centerOfRotation)
		secondHalfBb = self._getLineSegmentExpandedBb(secondHalfTransformation, edgeAtMid, secondHalfTransformation.rotation, centerOfRotation)
		return ((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb, edgeAtMid)

	def __checkIntervalsForOverlap(self, interval1: Tuple[float, float], interval2: Tuple[float, float]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	def __edgesHaveACommonVertex(self, l1: LineString, l2: LineString) -> bool:
		l1Verts = l1.coords
		l2Verts = l2.coords
		for v1 in l1Verts:
			for v2 in l2Verts:
				if Geometry.coordsAreAlmostEqual(v1, v2): return True
		return False

	def __splitIntervalsListForOverlap(self, intervals: List[Tuple[LineString, LineString, float, float]]) -> Tuple[List[Tuple[LineString, LineString, float, float]], List[Tuple[LineString, LineString, float, float]]]:
		haveOverlap = []
		dontHaveOverlap = []
		while len(intervals) > 0:
			interval1 = intervals.pop()
			foundOverlap = False
			for interval2 in intervals:
				if interval1 == interval2: continue
				if self.__checkIntervalsForOverlap(interval1[2:], interval2[2:]):
					if interval1[0] == interval2[0]:
						# If the sensor edges are the same
						if self.__edgesHaveACommonVertex(interval1[1], interval2[1]):
							# AND if the map edges are consecutive,
							# the sensor edge is going to collide both edges at the common vertex.
							# So we can safely remove one of them from the overlap
							continue
					foundOverlap = True
					haveOverlap.append(interval1)
					break
			if not foundOverlap:
				dontHaveOverlap.append(interval1)
		return (haveOverlap, dontHaveOverlap)

	def __expandVertObbWithAngularVelocity(self, coords: Geometry.Coords, angle: float, centerOfRotation: Geometry.Coords, expandAway = True) -> Tuple[float, float]:
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	def __initEdgeIntervals(self, sensor: SensorRegion, transformation: AffineTransform, centerOfRotation: Geometry.Coords, collisionData: Dict[str, Set[LineString]], inverse: bool):
		"""
			Initialize intervals for edges that are currently collisions with the sensor.
		"""
		intervals = []
		intStart = 1 if inverse else 0
		intEnd = 0 if inverse else 1
		for sensorEdgeId in collisionData:
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			sensorEdge = sensor.__edges[sensorEdgeId]
			for mapEdge in mapEdges:
				collisionCheckResults = self.__checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collisionCheckResults[0] == collisionCheckResults[1]: continue
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def __initCollisionIntervals(self, sensor: SensorRegion, collisionData: Dict[str, Set[LineString]]):
		intervals = []
		for sensorEdgeId in collisionData:
			sensorEdge = sensor.__edges[sensorEdgeId]
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			for mapEdge in mapEdges:
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def __findEventIntervalsForCollisions(self, previousSensor: SensorRegion, collisionIntervals: Dict[str, Set[LineString]], transformation: AffineTransform, centerOfRotation: Geometry.Coords, ingoingIntervals: list, outgoingIntervals: list):
		i = 0
		while i < len(collisionIntervals):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= self.MIN_TIME_DELTA:
				continue
			((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb, edgeAtMid) = self.__checkBoundingBoxIntervalForCollision(previousSensor, sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalEnd)
			intervalMid = (intervalStart + intervalEnd) / 2
			if isCollidingAtStart != isCollidingAtMid:
				if isCollidingAtStart and not isCollidingAtMid:
					outgoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
			if isCollidingAtMid != isCollidingAtEnd:
				if isCollidingAtMid and not isCollidingAtEnd:
					outgoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.append((sensorEdge, mapEdge, intervalStart, intervalMid))
			if isCollidingAtStart == isCollidingAtMid and isCollidingAtMid == isCollidingAtEnd:
				if firstHalfBb.intersects(mapEdge):
					collisionIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				if secondHalfBb.intersects(mapEdge):
					collisionIntervals.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
		return

	def __findIntermediateCollisionsWithMap(self, previousSensor: SensorRegion, currentSensor: SensorRegion, centerOfRotation: Geometry.Coords, mapPoly: Union[Polygon, MultiPolygon]) -> CtCd.CollisionEvent:
		"""
			Given the original configuration of the FOV (`previousSensor`) and the final configuration (`currentSensor`)
			find all the times where there is a shadow component event.

			That is, as the FOV moves towards its final configuration save the intermediate configurations for which there is a topological change (FOV hitting a "gap").
			#### Returns
			A list of intervals in each of which there is at most one component event.
		"""
		transformation = Geometry.getAffineTransformation(previousSensor.envelope, currentSensor.interior, centerOfRotation)
		previousCollidingEdges = self._getCollidingEdgesByEdge(previousSensor, mapPoly)
		currentCollidingEdges = self._getCollidingEdgesByEdge(currentSensor, mapPoly)
		intermediateCollisions = self._findCollisionsWithExtendedBb(previousSensor, transformation, centerOfRotation, mapPoly)
		# Remove the edges that we are sure are intersecting
		for id in previousCollidingEdges:
			for l in previousCollidingEdges[id]:
				if id in intermediateCollisions: intermediateCollisions[id].remove(l)
		for id in currentCollidingEdges:
			for l in currentCollidingEdges[id]:
				if id in intermediateCollisions: intermediateCollisions[id].remove(l)
		collisionIntervals = self._initCollisionIntervals(previousSensor, intermediateCollisions)
		# [self.lines.append(e[1]) for e in collisionIntervals]

		(ingoingIntervals, outgoingIntervals) = ([], [])
		while len(collisionIntervals) > 0:
			self._findEventIntervalsForCollisions(previousSensor, collisionIntervals, transformation, centerOfRotation, ingoingIntervals, outgoingIntervals)
		# Each interval is represented as a tuple: (sensorEdge, mapEdge, float, float)
		# The first float is the interval start and the second one is interval end times.
		intervals = self._initEdgeIntervals(previousSensor, transformation, centerOfRotation, previousCollidingEdges, inverse=False)
		haveOverlap = intervals + outgoingIntervals
		dontHaveOverlap = []
		eventCandidates = []
		i = 0
		while len(haveOverlap) > 0 and i < len(haveOverlap):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
			intervalMid = (intervalStart + intervalEnd) / 2
			collisionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalMid)
			if collisionCheckResults[0] != collisionCheckResults[1]:
				haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
				i += 1
			collisionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalMid, intervalEnd)
			if collisionCheckResults[0] != collisionCheckResults[1]:
				haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
				i += 1
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.interior, centerOfRotation)
				eventCandidates.append((p, interval[3], "outgoing", interval[1]))
		intervals = self._initEdgeIntervals(currentSensor, transformation, centerOfRotation, currentCollidingEdges, inverse=True)
		haveOverlap = intervals + ingoingIntervals
		dontHaveOverlap = []
		i = 0
		while len(haveOverlap) > 0 and i < len(haveOverlap):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
			intervalMid = (intervalStart + intervalEnd) / 2
			collisionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalMid)
			if collisionCheckResults[0] != collisionCheckResults[1]:
				haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
				i += 1
			collisionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalMid, intervalEnd)
			if collisionCheckResults[0] != collisionCheckResults[1]:
				haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
				i += 1
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.interior, centerOfRotation)
				eventCandidates.append((p, interval[3], "ingoing", interval[1]))
		# Sort events by time
		eventCandidates.sort(key=lambda e: e[1])
		# Remove duplicate times
		times = set()
		i = 0
		while i < len(eventCandidates):
			if eventCandidates[i][1] not in times:
				times.add(eventCandidates[i][1])
				i += 1
			else:
				eventCandidates.pop(i)
		# (FOV polygon, timeOfEvent, "ingoing" | "outgoing", map edge relevant to the event)
		return eventCandidates

	def __interpolateTrack(self, previousTracks: Tracklets, currentTracks: Tracklets, eventTime: float, eventFraction: float) -> Tracklets:
		"""
			this function assumes members in previousTracks and currentTracks all have the same timestamp.
		"""
		if len(previousTracks) == 0 or len(currentTracks) == 0: return currentTracks
		if eventFraction == 0: return previousTracks
		if eventFraction == 1: return currentTracks
		interpolatedTracks = {}
		(currentTime, _) = next(iter(currentTracks))
		for (prevTime, trackId) in previousTracks:
			previousPose = previousTracks[(prevTime, trackId)].pose
			currentPose = currentTracks[(currentTime, trackId)].pose
			x = ((eventFraction * (currentPose.x - previousPose.x)) + previousPose.x)
			y = ((eventFraction * (currentPose.y - previousPose.y)) + previousPose.y)
			psi = ((eventFraction * (currentPose.psi - previousPose.psi)) + previousPose.psi)
			interpolatedTracks[(eventTime, trackId)] = Track(trackId, eventTime, x, y, psi, isInterpolated=True)
		return interpolatedTracks

	def __appendConnectivityGraphPerEvent(self, regions: List[PolygonalRegion], events: CtCd.CollisionEvent, pastCGraph: ConnectivityGraph, nowSensors: Tracklets, symbols: Dict[str, Symbol], startTime: float, endTime: float) -> List[ConnectivityGraph]:
		graphs = []
		for event in events:
			eventTime = ((event[1] * (endTime - startTime)) + startTime)
			interpolatedTracks = self.__interpolateTrack(pastCGraph, nowSensors, eventTime, event[1])
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTime}
			graph = ConnectivityGraph(eventTime, regions, event[0], filteredTracks, symbols)
			graphs.append(graph)
		return graphs

	def __addTemporalEdges(self, eventGraphs: List[ConnectivityGraph], centerOfRotation: Geometry.Coords) -> None:
		for nowGraph in eventGraphs:
			isInitialGraph = (len(self.__history) == 0)
			self.__appendToHistory(nowGraph)
			if isInitialGraph: return
			pastGraph = self.__history[-2]
			# Add temporal edges between fovs
			for sensor in pastGraph.fov:
				fovNodeInShadowTree = self.generateTemporalName(sensor, pastGraph.timeNanoSecs)
				if fovNodeInShadowTree not in self.nodes: continue
				fovNodeInCurrentGraph = self.generateTemporalName(sensor, nowGraph.timeNanoSecs)
				if fovNodeInCurrentGraph not in self.nodes: continue
				self.__addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between symbols
			for symNode in pastGraph.symbolNodes:
				symNodeInShadowTree = self.generateTemporalName(symNode, pastGraph.time)
				symNodeInCurrentGraph = self.generateTemporalName(symNode, nowGraph.timeNanoSecs)
				self.__addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between shadows
			for shadowNodeInPastGraph in pastGraph.shadowNodes:
				for shadowNodeInNowGraph in nowGraph.__shadowNodes:
					previousShadowNodeRegion = pastGraph.nodes[shadowNodeInPastGraph]["region"]
					currentShadowNodeRegion = nowGraph.nodes[shadowNodeInNowGraph]["region"]
					if previousShadowNodeRegion.polygon.intersects(currentShadowNodeRegion.polygon):
						if self.__shadowsAreConnectedTemporally(pastGraph, nowGraph, pastGraph.nodes[shadowNodeInPastGraph], nowGraph.nodes[shadowNodeInNowGraph], centerOfRotation):
							shadowNodeInShadowTree = self.generateTemporalName(shadowNodeInPastGraph, pastGraph.timeNanoSecs)
							shadowNodeInNowGraph = self.generateTemporalName(shadowNodeInNowGraph, nowGraph.timeNanoSecs)
							self.nodes[shadowNodeInShadowTree]["toTime"] = nowGraph.timeNanoSecs
							self.__addEdge(shadowNodeInShadowTree, shadowNodeInNowGraph, isTemporal=True)
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		for node in graph.nodes:
			temporalName = self.generateTemporalName(node, graph.timeNanoSecs)
			self.__addNode(temporalName, graph.timeNanoSecs)
			self.shallowCopyNode(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self.generateTemporalName(edge[0], graph.timeNanoSecs)
			to = self.generateTemporalName(edge[1], graph.timeNanoSecs)
			self.__addEdge(frm, to, False)
		self.__history.append(graph)
		return

	def bfs(self, start: str, goalFunc: NfaLambda.NfaLambdaFunc) -> Union[List[str], None]:
		q = Queue()
		visited = set()
		q.put([start])
		while not q.empty():
			path = q.get()
			n = path[-1]
			visited.add(n)
			nodeData = self.nodes[n]
			spaceTimeSetOfNode = ProjectiveSpaceTimeSet(nodeData["region"].interior, TimeInterval(nodeData["fromTime"], nodeData["toTime"], True, True))
			# if nodeData["type"] == "sensor": continue # FIXME: check if there is a track head here
			if goalFunc(spaceTimeSetOfNode):
				return path
			for child in self.adj[n]:
				if child in visited: continue
				newPath = list(path)
				newPath.append(child)
				q.put(newPath)
		return None

	def updateNamedRegions(self, timeNanoSecs: int, regions: List[SymbolRegion]) -> None:
		return
		if self.__history.isEmpty:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs, regions, Polygon(), dict(), validators)
			self.__appendToHistory(currentConnectivityG)
			previousFov = currentFov
		else:
			componentEvents = currentFov.estimateIntermediateCollisionsWithPolygon(previousFov, mapPolygon)
			graphs = self.__appendConnectivityGraphPerEvent(envMap, componentEvents, self.previousTracks, filteredTracks, validators, previousFov.time, currentFov.time)
			i=0
		self.previousTracks = filteredTracks
		# graphPacks.sort(key=lambda x:x[0].time)
		# for (graph, centerOfRotation) in graphPacks:
		# 	self._addTemporalEdges(graph, centerOfRotation)
		return

	def updateSensors(self, timeNanoSecs: int, regions: List[AffineSensorRegion], symbols: Dict[str, Symbol] = {}) -> None:
		if len(self.__history) == 0:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, regions=regions)
			RosUtils.Logger().info("Appending initial CGraph to Shadow Tree.")
			self.__appendToHistory(currentConnectivityG)
		else:
			pastCGraph = self.__history[-1]
			nowFov = FieldOfView(regions)
			events = CtCd.estimateIntermediateCollisionsWithPolygon(pastCGraph.fov, nowFov, pastCGraph.mapPerimeter)
			RosUtils.Logger().info("Appending CGraphs to Shadow Tree for %d events." % len(events))
			eventGraphs = self.__appendConnectivityGraphPerEvent(
				regions=regions,
				events=events,
				pastCGraph=pastCGraph,
				nowSensors=nowFov,
				symbols=symbols,
				startTime=pastCGraph.timeNanoSecs,
				endTime=timeNanoSecs
			)
			for g in eventGraphs: self.__appendToHistory(g)
		# graphPacks.sort(key=lambda x:x[0].time)
		# for (graph, centerOfRotation) in graphPacks:
		# 	self._addTemporalEdges(graph, centerOfRotation)
		return

	def updateMap(self, timeNanoSecs: int, regions: List[MapRegion]) -> None:
		if len(self.__history) == 0:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, regions=regions)
			self.__appendToHistory(currentConnectivityG)
		return
		componentEvents = currentFov.estimateIntermediateCollisionsWithPolygon(previousFov, mapPolygon)
		graphs = self.__appendConnectivityGraphPerEvent(envMap, componentEvents, self.previousTracks, filteredTracks, validators, previousFov.time, currentFov.time)
		i=0
		self.previousTracks = filteredTracks
		# graphPacks.sort(key=lambda x:x[0].time)
		# for (graph, centerOfRotation) in graphPacks:
		# 	self._addTemporalEdges(graph, centerOfRotation)
		return

	def render(self) -> None:
		"""
		Detects beam and non-beam nodes for the correct coloring
		"""
		self.renderSpringGraph(self.history[0].fov.regionNames, redNodes, symNodes)

	def renderSpringGraph(self, greenNodes: List[str], redNodes: List[str], symbolNodes: List[str]) -> None:
		greenNodesWithSymbols = greenNodes.copy()
		redNodesWithSymbols = redNodes.copy()
		for symNode in symbolNodes:
			if self.nodes[symNode]["region"].inFov:
				greenNodesWithSymbols.append(symNode)
			else:
				redNodesWithSymbols.append(symNode)
		if self.__fig is None:
			fig = figure(self.name)
			fig.set_facecolor("grey") # cspell: disable-line
			axis("off")
		else:
			fig = self.__fig
		# pos = nx.spring_layout(g)
		# pos = nx.kamada_kawai_layout(g, scale=-1) # cspell: disable-line
		# pos = nx.kamada_kawai_layout(g, weight="fromTime", scale=-1) # cspell: disable-line
		# pos = nx.multipartite_layout(g, subset_key="fromTime", align="horizontal")

		graphs = self.history if hasattr(self, "history") else tuple([self])
		pos = self.multiPartiteLayout(graphs)
		nx.draw_networkx_nodes(self, pos, nodelist=greenNodesWithSymbols, node_color="PaleGreen")
		nx.draw_networkx_nodes(self, pos, nodelist=redNodesWithSymbols, node_color="Tomato")
		normalEdges = []
		temporalEdges = []
		for e in self.edges:
			if "isTemporal" in self.edges[e] and self.edges[e]["isTemporal"]:
				temporalEdges.append(e)
			else:
				normalEdges.append(e)
		nx.draw_networkx_edges(self, pos, edgelist=normalEdges, edge_color="blue") # cspell: disable-line
		nx.draw_networkx_edges(self, pos, edgelist=temporalEdges, edge_color="red", width=2) # cspell: disable-line
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		pause(0.05)
		return

	def killDisplayedGraph(self) -> None:
		if self.__fig:
			close(self.__fig)
			self.__fig = None

	@staticmethod
	def shallowCopyNode(frm: dict, to: dict) -> None:
		for key in frm: to[key] = frm[key]
		return

	@staticmethod
	def generateTemporalName(name: str, time: int) -> str:
		""" Use this function to safely generate (typo-free and in uniform format) the names for the temporal edges. """
		return "%s-%d" % (name, time)
