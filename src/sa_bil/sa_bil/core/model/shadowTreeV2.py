import networkx as nx
from shapely.geometry import LineString, Polygon, MultiPolygon
from skimage import transform
import time
from typing import List, Set, Dict, Tuple, Union
from sa_bil.core.model.sensingRegion import SensingRegion
from sa_bil.core.model.connectivityGraph import ConnectivityGraph
from sa_bil.core.model.map import Map
from sa_bil.core.observation.fov import Fov, Events
from sa_bil.core.observation.track import Track, Tracks
from sa_bil.core.spec.validator import Validator
from sa_bil.core.utils.geometry import Geometry
from sa_bil.core.utils.graph import GraphAlgorithms


class ShadowTreeV2(nx.DiGraph):
	def __init__(self):
		super().__init__()
		self.MIN_TIME_DELTA = 1E-2
		self._fig = None
		# The reason this is a list of lists is that the time of event is relative to the time between
		self.componentEvents: List[List[Polygon]] = []
		self.graphs: List[ConnectivityGraph] = []
		# Used for debugging
		self.redPolys: List[Polygon] = []
		self.bluePolys: List[Polygon] = []
		self.lines: List[LineString] = []

	def __repr__(self) -> str:
		return "ShadowTreeV2"

	def _generateTemporalName(self, name: str, time: float):
		return "%s-%.2f" % (name, time)

	def _getLowerAndUpperNode(self, n1, n2):
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def _addNode(self, n, time):
		self.add_node(n)
		self.nodes[n]["fromTime"] = time
		self.nodes[n]["toTime"] = time # This will be updated accordingly when adding temporal edges

	def _addEdge(self, n1: str, n2: str, isTemporal: bool, fromTime = None, toTime = None):
		(lower, upper) = self._getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=isTemporal, fromTime=fromTime, toTime=toTime)

	def _shadowsAreConnectedTemporally(self, previousGraph: ConnectivityGraph, currentGraph: ConnectivityGraph, previousShadow: dict, currentShadow: dict, centerOfRotation: Geometry.Coords):
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

	def _checkEdgeIntervalForCollision(self, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[bool, bool]:
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		movingEdgeStartConfiguration = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		movingEdgeEndConfiguration = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		return (movingEdgeStartConfiguration.intersects(staticEdge), movingEdgeEndConfiguration.intersects(staticEdge))

	def _checkBoundingBoxIntervalForCollision(self, sensor: SensingRegion, movingEdge: LineString, staticEdge: LineString, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, intervalStart: float, intervalEnd: float) -> Tuple[Tuple[bool, bool, bool], Polygon, Polygon, LineString]:
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

	def _checkIntervalsForOverlap(self, interval1: Tuple[float, float], interval2: Tuple[float, float]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	def _edgesHaveACommonVertex(self, l1: LineString, l2: LineString):
		l1Verts = l1.coords
		l2Verts = l2.coords
		for v1 in l1Verts:
			for v2 in l2Verts:
				if Geometry.coordsAreAlmostEqual(v1, v2): return True
		return False

	def _splitIntervalsListForOverlap(self, intervals: List[Tuple[LineString, LineString, float, float]]) -> Tuple[List[Tuple[LineString, LineString, float, float]], List[Tuple[LineString, LineString, float, float]]]:
		haveOverlap = []
		dontHaveOverlap = []
		while len(intervals) > 0:
			interval1 = intervals.pop()
			foundOverlap = False
			for interval2 in intervals:
				if interval1 == interval2: continue
				if self._checkIntervalsForOverlap(interval1[2:], interval2[2:]):
					if interval1[0] == interval2[0]:
						# If the sensor edges are the same
						if self._edgesHaveACommonVertex(interval1[1], interval2[1]):
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

	def _expandVertObbWithAngularVelocity(self, coords: Geometry.Coords, angle: float, centerOfRotation: Geometry.Coords, expandAway = True):
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	def _initEdgeIntervals(self, sensor: SensingRegion, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, collisionData: Dict[str, Set[LineString]], inverse: bool):
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
				collisionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collisionCheckResults[0] == collisionCheckResults[1]: continue
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def _initCollisionIntervals(self, sensor: SensingRegion, collisionData: Dict[str, Set[LineString]]):
		intervals = []
		for sensorEdgeId in collisionData:
			sensorEdge = sensor.edges[sensorEdgeId]
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			for mapEdge in mapEdges:
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	def _findEventIntervalsForCollisions(self, previousSensor: SensingRegion, collisionIntervals: Dict[str, Set[LineString]], transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, ingoingIntervals: list, outgoingIntervals: list):
		i = 0
		while i < len(collisionIntervals):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= self.MIN_TIME_DELTA:
				continue
			((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb, edgeAtMid) = self._checkBoundingBoxIntervalForCollision(previousSensor, sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalEnd)
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

	def _findIntermediateCollisionsWithMap(self, previousFov: Fov, currentFov: Fov, envMap: Map) -> Events:
		"""
			Given the original configuration of the FOV and the final configuration of it,
			find all the times when there is a shadow component event.

			That is, as the FOV moves towards its final configuration save the intermediate configurations for which there is a topological change (FOV hitting a "gap").
			#### Returns
			A list of intervals in each of which there is at most one component event.
		"""
		for sensorId in previousFov.sensors:#
			collisionData = collisionData | previousFov.sensors[sensorId].findCollisionsWithExtendedBb(envMap.polygon)#
			collisionData = collisionData | currentFov.sensors[sensorId].findCollisionsWithExtendedBb(envMap.polygon)#
			previousCollidingEdgesByEdge = self._getCollidingEdgesByEdge(previousSensor.region, envMap.polygon)
			currentCollidingEdgesByEdge = self._getCollidingEdgesByEdge(currentSensor.region, envMap.polygon)
			transformation = Geometry.getAffineTransformation(previousSensor.region.polygon, currentSensor.region.polygon, centerOfRotation)
			intermediateCollisions = self._findCollisionsWithExtendedBb(previousSensor.region, transformation, centerOfRotation, envMap.polygon)
			# Remove the edges that we are sure are intersecting
			for id in previousCollidingEdgesByEdge:
				for l in previousCollidingEdgesByEdge[id]:
					if id in intermediateCollisions: intermediateCollisions[id].remove(l)
			for id in currentCollidingEdgesByEdge:
				for l in currentCollidingEdgesByEdge[id]:
					if id in intermediateCollisions: intermediateCollisions[id].remove(l)
			collisionIntervals = self._initCollisionIntervals(previousSensor.region, intermediateCollisions)
			# [self.lines.append(e[1]) for e in collisionIntervals]

			(ingoingIntervals, outgoingIntervals) = ([], [])
			while len(collisionIntervals) > 0:
				self._findEventIntervalsForCollisions(previousSensor.region, collisionIntervals, transformation, centerOfRotation, ingoingIntervals, outgoingIntervals)
			# Each interval is represented as a tuple: (sensorEdge, mapEdge, float, float)
			# The first float is the interval start and the second one is interval end times.
			intervals = self._initEdgeIntervals(previousSensor.region, transformation, centerOfRotation, previousCollidingEdgesByEdge, inverse=False)
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
					p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.region.polygon, centerOfRotation)
					eventCandidates.append((p, interval[3], "outgoing", interval[1]))
			intervals = self._initEdgeIntervals(currentSensor.region, transformation, centerOfRotation, currentCollidingEdgesByEdge, inverse=True)
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
					p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.region.polygon, centerOfRotation)
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

	def _interpolateTrack(self, previousTracks: Tracks, currentTracks: Tracks, eventTime: float, eventFraction: float) -> Tracks:
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

	def _appendConnectivityGraphPerEvent(self, envMap: Map, events: Events, previousTracks: Tracks, currentTracks: Tracks, validators: Dict[str, Validator], startTime: float, endTime: float):
		graphs = []
		for event in events:
			eventTime = ((event[1] * (endTime - startTime)) + startTime)
			interpolatedTracks = self._interpolateTrack(previousTracks, currentTracks, eventTime, event[1])
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTime}
			graph = ConnectivityGraph(envMap, event[0], filteredTracks, eventTime, validators)
			graphs.append(graph)
		return graphs

	def _addTemporalEdges(self, eventGraphs: List[ConnectivityGraph], centerOfRotation: Geometry.Coords):
		for graph in eventGraphs:
			isInitialGraph = (len(self.graphs) == 0)
			self._appendGraph(graph)
			if isInitialGraph: return
			previousGraph = self.graphs[-2]
			# Add temporal edges between fovs
			for fovNode in previousGraph.fovNodes:
				fovNodeInShadowTree = self._generateTemporalName(fovNode, previousGraph.time)
				if fovNodeInShadowTree not in self.nodes: continue
				fovNodeInCurrentGraph = self._generateTemporalName(fovNode, graph.time)
				if fovNodeInCurrentGraph not in self.nodes: continue
				self._addEdge(fovNodeInShadowTree, fovNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between symbols
			for symNode in previousGraph.symbolNodes:
				symNodeInShadowTree = self._generateTemporalName(symNode, previousGraph.time)
				symNodeInCurrentGraph = self._generateTemporalName(symNode, graph.time)
				self._addEdge(symNodeInShadowTree, symNodeInCurrentGraph, isTemporal=True)
			# Add temporal edges between shadows
			for shadowNodeInPreviousGraph in previousGraph.shadowNodes:
				for shadowNodeInCurrentGraph in graph.shadowNodes:
					previousShadowNodeRegion = previousGraph.nodes[shadowNodeInPreviousGraph]["region"]
					currentShadowNodeRegion = graph.nodes[shadowNodeInCurrentGraph]["region"]
					if previousShadowNodeRegion.polygon.intersects(currentShadowNodeRegion.polygon):
						if self._shadowsAreConnectedTemporally(previousGraph, graph, previousGraph.nodes[shadowNodeInPreviousGraph], graph.nodes[shadowNodeInCurrentGraph], centerOfRotation):
							shadowNodeInShadowTree = self._generateTemporalName(shadowNodeInPreviousGraph, previousGraph.time)
							shadowNodeInCurrentGraph = self._generateTemporalName(shadowNodeInCurrentGraph, graph.time)
							self.nodes[shadowNodeInShadowTree]["toTime"] = graph.time
							self._addEdge(shadowNodeInShadowTree, shadowNodeInCurrentGraph, isTemporal=True)
		return

	def _appendGraph(self, graph: ConnectivityGraph):
		for node in graph.nodes:
			temporalName = self._generateTemporalName(node, graph.time)
			self._addNode(temporalName, graph.time)
			GraphAlgorithms.cloneNodeProps(graph.nodes[node], self.nodes[temporalName])
		for edge in graph.edges:
			frm = self._generateTemporalName(edge[0], graph.time)
			to = self._generateTemporalName(edge[1], graph.time)
			self._addEdge(frm, to, False)
		self.graphs.append(graph)
		return

	def _buildViaStream(self, envMap: Map, fovs: List[Fov], validators: Dict[str, Validator], tracks: Tracks, startInd = 0, endInd = None):
		if endInd is None: endInd = len(fovs)

		previousFov: Fov = None
		currentConnectivityG: ConnectivityGraph = None
		startTime = time.time()
		previousTracks = None

		for i in range(startInd, endInd):
			currentFov = fovs[i]
			filteredTracks = {(tTime, tId): tracks[(tTime, tId)] for (tTime, tId) in tracks if tTime == currentFov.time}
			if previousFov is None:
				currentConnectivityG = ConnectivityGraph(envMap, currentFov.polygon, filteredTracks, currentFov.time, validators)
				self._appendGraph(currentConnectivityG)
				previousFov = currentFov
			else:
				for sensorId in previousFov.sensors:
					previousSensor = previousFov.sensors[sensorId]
					currentSensor = currentFov.getEquivalentSensorById(sensorId)
					centerOfRotation = (previousSensor.pose.x, previousSensor.pose.y)
					componentEvents = self._findIntermediateCollisionsWithMap(previousSensor.region, currentSensor.region, centerOfRotation, envMap)
					self.componentEvents.append(componentEvents)
					graphs = self._appendConnectivityGraphPerEvent(envMap, componentEvents, previousTracks, filteredTracks, validators, previousFov.time, currentFov.time)
					self._addTemporalEdges(graphs, centerOfRotation)
				previousTracks = filteredTracks
			previousFov = currentFov
		print("took %.2fms" % (time.time() - startTime))
		return

	def _update(self, envMap: Map, previousFov: Fov, currentFov: Fov, validators: Dict[str, Validator], tracks: Tracks):
		startTime = time.time()
		# this is here to interpolate the location of the target as well if it is in fov. FIXME: This should be resolved later
		filteredTracks = {(tTime, tId): tracks[(tTime, tId)] for (tTime, tId) in tracks if tTime == currentFov.time}
		if previousFov is None:
			currentConnectivityG = ConnectivityGraph(envMap, currentFov.polygon, filteredTracks, currentFov.time, validators)
			self._appendGraph(currentConnectivityG)
			previousFov = currentFov
		else:
			componentEvents = currentFov.estimateIntermediateCollisionsWithPolygon(previousFov, envMap.polygon)
			graphs = self._appendConnectivityGraphPerEvent(envMap, componentEvents, self.previousTracks, filteredTracks, validators, previousFov.time, currentFov.time)
			i=0
		self.previousTracks = filteredTracks
		# graphPacks.sort(key=lambda x:x[0].time)
		# for (graph, centerOfRotation) in graphPacks:
		# 	self._addTemporalEdges(graph, centerOfRotation)
		print("took %.2fms" % (time.time() - startTime))
		return

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		self._fig = GraphAlgorithms.displayGraphAuto(self, displayGeomGraph, displaySpringGraph)

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
