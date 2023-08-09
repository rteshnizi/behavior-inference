import time
from typing import Dict, List, Set, Tuple, Union

import networkx as nx
from skimage import transform

from rt_bi_core.Model import Events, Fov, MapRegion, PolygonalRegion, SensingRegion, Tracks
from rt_bi_core.ShadowTree.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.BehaviorAutomaton.Validator import Validator
from rt_bi_utils.Geometry import Geometry, LineString, Polygon, MultiPolygon
from rt_bi_core.ShadowTree.Graph import GraphAlgorithms
from rt_bi_utils.MinQueue import MinQueue


class ShadowTree(nx.DiGraph):
	"""The implementation of a Shadow Tree in python as described in the dissertation of Reza Teshnizi."""
	def __init__(self):
		"""Initialize the shadow tree. The tree is expected to be updated in a streaming fashion."""
		super().__init__()
		# The reason this is a list of lists is that the time of event is relative to the time between
		self.componentEvents: List[List[Polygon]] = []
		__data: List[ConnectivityGraph] = []
		self.__history: MinQueue = MinQueue(key=lambda g: -1 * g.timeNanoSecs, initial=__data)
		self.__mapRegions: List[MapRegion] = []
		self.__sensingRegions: List[SensingRegion] = []
		# Used for debugging
		self.redPolys: List[Polygon] = []
		self.bluePolys: List[Polygon] = []
		self.lines: List[LineString] = []

	@property
	def MIN_TIME_DELTA(self) -> float:
		"""
		### A Core Assumption:
		We expect the updates to be at least as fast as 1 ns.
		"""
		return 1

	def __repr__(self) -> str:
		return "ShadowTree v3"

	def __generateTemporalName(self, name: str, time: float) -> str:
		""" Use this function to safely generate (typo-free and in uniform format) the names for the temporal edges. """
		return "%s-%.2f" % (name, time)

	def __getLowerAndUpperNode(self, n1: str, n2: str) -> Tuple[str, str]:
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def __addNode(self, n: str, timeNanoSecs: float) -> None:
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
		for fov in previousGraph.fovNodes:
			if fov not in previousGraph.nodes: continue
			previousFovRegion: SensingRegion = previousGraph.nodes[fov]["region"]
			if fov not in currentGraph.nodes: continue
			currentFovRegion: SensingRegion = currentGraph.nodes[fov]["region"]
			transformation: transform.AffineTransform = Geometry.getAffineTransformation(previousFovRegion.polygon, currentFovRegion.polygon, centerOfRotation)
			ps = []
			for e in currentFovRegion.edges:
				edgeC = currentFovRegion.edges[e]
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

	def __checkEdgeIntervalForCollision(self, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[bool, bool]:
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		movingEdgeStartConfiguration = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		movingEdgeEndConfiguration = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		return (movingEdgeStartConfiguration.intersects(staticEdge), movingEdgeEndConfiguration.intersects(staticEdge))

	def __checkBoundingBoxIntervalForCollision(self, sensor: SensingRegion, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[Tuple[bool, bool, bool], Polygon, Polygon, LineString]:
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
		startSensor = Geometry.applyMatrixTransformToPolygon(startConfigTransformation, sensor.polygon, centerOfRotation)
		midSensor = Geometry.applyMatrixTransformToPolygon(midConfigTransformation, sensor.polygon, centerOfRotation)
		endSensor = Geometry.applyMatrixTransformToPolygon(endConfigTransformation, sensor.polygon, centerOfRotation)
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

	def __initEdgeIntervals(self, sensor: SensingRegion, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, collisionData: Dict[str, Set[LineString]], inverse: bool):
		"""
			Initialize intervals for edges that are currently collisions with the sensor.
		"""
		intervals = []
		intStart = 1 if inverse else 0
		intEnd = 0 if inverse else 1
		for sensorEdgeId in collisionData:
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			sensorEdge = sensor.edges[sensorEdgeId]
			for mapEdge in mapEdges:
				collisionCheckResults = self.__checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collisionCheckResults[0] == collisionCheckResults[1]: continue
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def __initCollisionIntervals(self, sensor: SensingRegion, collisionData: Dict[str, Set[LineString]]):
		intervals = []
		for sensorEdgeId in collisionData:
			sensorEdge = sensor.edges[sensorEdgeId]
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			for mapEdge in mapEdges:
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def __findEventIntervalsForCollisions(self, previousSensor: SensingRegion, collisionIntervals: Dict[str, Set[LineString]], transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, ingoingIntervals: list, outgoingIntervals: list):
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

	def __findIntermediateCollisionsWithMap(self, previousSensor: SensingRegion, currentSensor: SensingRegion, centerOfRotation: Geometry.Coords, mapPoly: Union[Polygon, MultiPolygon]) -> Events:
		"""
			Given the original configuration of the FOV (`previousSensor`) and the final configuration (`currentSensor`)
			find all the times where there is a shadow component event.

			That is, as the FOV moves towards its final configuration save the intermediate configurations for which there is a topological change (FOV hitting a "gap").
			#### Returns
			A list of intervals in each of which there is at most one component event.
		"""
		transformation = Geometry.getAffineTransformation(previousSensor.polygon, currentSensor.polygon, centerOfRotation)
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
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.polygon, centerOfRotation)
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
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.polygon, centerOfRotation)
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

	def __interpolateTrack(self, previousTracks: Tracks, currentTracks: Tracks, eventTime: float, eventFraction: float) -> Tracks:
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

	def __appendConnectivityGraphPerEvent(self, regions: List[PolygonalRegion], events: Events, previousTracks: Tracks, currentTracks: Tracks, validators: Dict[str, Validator], startTime: float, endTime: float) -> List[ConnectivityGraph]:
		graphs = []
		for event in events:
			eventTime = ((event[1] * (endTime - startTime)) + startTime)
			interpolatedTracks = self.__interpolateTrack(previousTracks, currentTracks, eventTime, event[1])
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTime}
			graph = ConnectivityGraph(eventTime, regions, event[0], filteredTracks, validators)
			graphs.append(graph)
		return graphs

	def __addTemporalEdges(self, eventGraphs: List[ConnectivityGraph], centerOfRotation: Geometry.Coords) -> None:
		for graph in eventGraphs:
			isInitialGraph = (len(self.__history) == 0)
			self.__appendToHistory(graph)
			if isInitialGraph: return
			previousGraph = self.__history[-2]
			# Add temporal edges between fovs
			for fovNode in previousGraph.fovNodes:
				fovNodeInShadowTree = self.__generateTemporalName(fovNode, previousGraph.time)
				if fovNodeInShadowTree not in self.nodes: continue
				fovNodeInCurrentGraph = self.__generateTemporalName(fovNode, graph.time)
				if fovNodeInCurrentGraph not in self.nodes: continue
				self.__addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between symbols
			for symNode in previousGraph.symbolNodes:
				symNodeInShadowTree = self.__generateTemporalName(symNode, previousGraph.time)
				symNodeInCurrentGraph = self.__generateTemporalName(symNode, graph.time)
				self.__addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between shadows
			for shadowNodeInPreviousGraph in previousGraph.shadowNodes:
				for shadowNodeInCurrentGraph in graph.shadowNodes:
					previousShadowNodeRegion = previousGraph.nodes[shadowNodeInPreviousGraph]["region"]
					currentShadowNodeRegion = graph.nodes[shadowNodeInCurrentGraph]["region"]
					if previousShadowNodeRegion.polygon.intersects(currentShadowNodeRegion.polygon):
						if self.__shadowsAreConnectedTemporally(previousGraph, graph, previousGraph.nodes[shadowNodeInPreviousGraph], graph.nodes[shadowNodeInCurrentGraph], centerOfRotation):
							shadowNodeInShadowTree = self.__generateTemporalName(shadowNodeInPreviousGraph, previousGraph.time)
							shadowNodeInCurrentGraph = self.__generateTemporalName(shadowNodeInCurrentGraph, graph.time)
							self.nodes[shadowNodeInShadowTree]["toTime"] = graph.time
							self.__addEdge(shadowNodeInShadowTree, shadowNodeInCurrentGraph, isTemporal=True)
		return

	def __appendToHistory(self, graph: ConnectivityGraph) -> None:
		for node in graph.nodes:
			temporalName = self.__generateTemporalName(node, graph.time)
			self.__addNode(temporalName, graph.time)
			GraphAlgorithms.cloneNodeProps(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self.__generateTemporalName(edge[0], graph.timeNanoSecs)
			to = self.__generateTemporalName(edge[1], graph.timeNanoSecs)
			self.__addEdge(frm, to, False)
		self.__history.enqueue(graph)
		return

	def updateNamedRegions(self, timeNanoSecs: float, regions: List[SensingRegion]) -> None:
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
		print("took %.2fms" % (time.time() - startTime))
		return

	def updateSensingRegions(self, timeNanoSecs: float, regions: List[SensingRegion]) -> None:
		if self.__history.isEmpty:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs=timeNanoSecs, regions=regions, fovUnion=Polygon(), tracks=dict(), validators=dict())
			self.__appendToHistory(currentConnectivityG)
		else:
			currentFov = self.__history
			componentEvents = currentFov.estimateIntermediateCollisionsWithPolygon(previousFov, mapPolygon)
			graphs = self.__appendConnectivityGraphPerEvent(envMap, componentEvents, self.previousTracks, filteredTracks, validators, previousFov.time, currentFov.time)
			i=0
		self.previousTracks = filteredTracks
		# graphPacks.sort(key=lambda x:x[0].time)
		# for (graph, centerOfRotation) in graphPacks:
		# 	self._addTemporalEdges(graph, centerOfRotation)
		print("took %.2fms" % (time.time() - startTime))
		return

	def updateMap(self, timeNanoSecs: float, regions: List[MapRegion]) -> None:
		return
		if self.__history.isEmpty:
			currentConnectivityG = ConnectivityGraph(timeNanoSecs, regions, currentFov.polygon, filteredTracks, currentFov.time, validators)
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
		print("took %.2fms" % (time.time() - startTime))
		return

	def displayGraph(self, displayGeomGraph, displaySpringGraph) -> None:
		self._fig = GraphAlgorithms.displayGraphAuto(self, displayGeomGraph, displaySpringGraph)

	def killDisplayedGraph(self) -> None:
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
