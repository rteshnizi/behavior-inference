import time
from typing import Dict, List, Set, Tuple

import networkx as nx
from shapely.geometry import LineString, Polygon
from skimage import transform

from rt_bi_core.Model.SensingRegion import SensingRegion
from rt_bi_core.ShadowTree.ConnectivityGraph import ConnectivityGraph
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Graph import GraphAlgorithms


class TimedGraph(nx.DiGraph):
	def __init__(self, graphs: List[ConnectivityGraph], startInd = 0, endInd = None):
		super().__init__()
		self.MIN_TIME_DELTA = 1E-2
		self._nodeLayers = []
		self._eventsByLayer = []
		self.nodeByLayers = []
		self.eventsByLayer = []
		self._timeStamps = []
		self._regionNodes = []
		self._beamNodes = []
		self._beamSet = set()
		self._boundary = None
		# TODO: DEBUGGING
		self.ingoingIntervals = []
		self.outgoingIntervals = []

		print("Connecting Graphs through time...")
		self._fig = None
		self.nodeClusters: Dict[str, Set[str]] = {}
		self.nodeToClusterMap: Dict[str, str] = {}
		self._build(graphs, startInd, endInd)

	def _getLowerAndUpperNode(self, n1, n2):
		upper = n1 if self.nodes[n1]["fromTime"] > self.nodes[n2]["fromTime"] else n2
		lower = n1 if upper != n1 else n2
		return (lower, upper)

	def _addTemporalEdge(self, n1, n2):
		(lower, upper) = self._getLowerAndUpperNode(n1, n2)
		self.add_edge(lower, upper, isTemporal=True)

	def _checkChangesInCollidingEdges(self, previousFovPolygon: Polygon, currentFovPolygon: Polygon, envMap: Map) -> Tuple[bool, Set[Geometry.CoordsList], Set[Geometry.CoordsList]]:
		"""
			Takes two connectivity graphs and looks at their respective FOVs
			to see if the edges of the FOV that collides crosses a different set of edges of the map's boundary.

			#### Return
			A tuple with three elements:
				(Whether the edges are the same or neighboring,
				The edges hitting the previous FOV configuration,
				The edges hitting the current FOV configuration)
		"""
		fovVerts = list(previousFovPolygon.exterior.coords)
		previousEdges = set()
		for v1, v2 in zip(fovVerts, fovVerts[1:]):
			line = LineString([v1, v2])
			edges = Geometry.getAllIntersectingEdgesWithLine(line, envMap.polygon)
			previousEdges.update(edges)
		fovVerts = list(currentFovPolygon.exterior.coords)
		currentEdges = set()
		for v1, v2 in zip(fovVerts, fovVerts[1:]):
			line = LineString([v1, v2])
			edges = Geometry.getAllIntersectingEdgesWithLine(line, envMap.polygon)
			currentEdges.update(edges)
		prevHash = [Geometry.coordListStringId(edge) for edge in previousEdges]
		currHash = [Geometry.coordListStringId(edge) for edge in currentEdges]
		edgesAreTheSame = prevHash == currHash
		if edgesAreTheSame: return (True, previousEdges, currentEdges)
		# # Are they almost the same (neighboring)
		# for edge1coords in previousEdges:
		# 	pts1Set = set(edge1coords)
		# 	for edge2coords in currentEdges:
		# 		pts2Set = set(edge2coords)
		# 		coordsInCommon = pts1Set.intersection(pts2Set)
		# 		# Common edge doesn't count
		# 		if len(coordsInCommon) == 0:
		# 			continue
		# 		if len(coordsInCommon) == 2:
		# 			break
		# 		if len(coordsInCommon) == 1:
		# 			# There is a neghboring edge
		# 			return (True, previousEdges, currentEdges)
		# 		raise("Past Reza said, check when does this happen")
		return (False, previousEdges, currentEdges)

	def _getCollidingEdgesByEdge(self, sensor: SensingRegion, envMap: Map) -> Dict[str, Set[LineString]]:
		collisionData = {}
		for sensorEdgeId in sensor.edges:
			collisionData[sensorEdgeId] = Geometry.getAllIntersectingEdgesWithLine(sensor.edges[sensorEdgeId], envMap.polygon)
		return collisionData

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

	def _getLineSegmentExpandedBb(self, transformation: transform.AffineTransform, lineSeg: LineString, angle: float, centerOfRotation: Geometry.Coords) -> Polygon:
		"""
			Gets a tight bounding box for a line segment that is moving with a constant angular velocity.
		"""
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg, centerOfRotation)
		polygons = []
		for j in range(16):
			v1 = self._expandVertObbWithAngularVelocity(lineSeg.coords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = self._expandVertObbWithAngularVelocity(finalConfig.coords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = self._expandVertObbWithAngularVelocity(finalConfig.coords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = self._expandVertObbWithAngularVelocity(lineSeg.coords[1], angle, centerOfRotation, j & 8 != 0)
			p = Polygon([v1, v2, v3, v4])
			polygons.append(p)
		expandedObb = Geometry.union(polygons)
		expandedObb = expandedObb.convex_hull
		return expandedObb

	def _findCollisionsWithExtendedBb(self, sensor: SensingRegion, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, envMap: Map) -> Dict[str, Set[LineString]]:
		"""
			#### Returns
				For each edge of the sensor, it returns all the edges of the map that intersect the expanded bounding box of that edge
		"""
		collisionData = {}
		angle = abs(transformation.rotation)
		for sensorEdgeId in sensor.edges:
			collisionData[sensorEdgeId] = []
			edge = sensor.edges[sensorEdgeId]
			boundingBox = self._getLineSegmentExpandedBb(transformation, edge, angle, centerOfRotation)
			mapBoundaryVerts = envMap.polygon.exterior.coords
			for v1, v2 in zip(mapBoundaryVerts, mapBoundaryVerts[1:]):
				mapEdge = LineString([v1, v2])
				if boundingBox.intersects(mapEdge): collisionData[sensorEdgeId].append(mapEdge)
		return collisionData

	def _initEdgeIntervals(self, sensor: SensingRegion, transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords, collisionData: Dict[str, Set[LineString]], inverse: bool):
		"""
			Initialize intervals for edges that are currently collising with the sensor.
		"""
		intervals = []
		intStart = 1 if inverse else 0
		intEnd = 0 if inverse else 1
		for sensorEdgeId in collisionData:
			mapEdges = collisionData[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			sensorEdge = sensor.edges[sensorEdgeId]
			for mapEdge in mapEdges:
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collsionCheckResults[0] == collsionCheckResults[1]: continue
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

	def _findEventIntervalsForCollisions(self, previousSensor: SensingRegion, collisionIntervals: Dict[str, Set[LineString]], transformation: transform.AffineTransform, centerOfRotation: Geometry.Coords):
		i = 0
		while i < len(collisionIntervals):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= self.MIN_TIME_DELTA:
				continue
			ingoingIntervals = []
			outgoingIntervals = []
			((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb, edgeAtMid) = self._checkBoundingBoxIntervalForCollision(previousSensor, sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalEnd)
			intervalMid = (intervalStart + intervalEnd) / 2
			if isCollidingAtStart != isCollidingAtMid:
				if isCollidingAtStart and not isCollidingAtMid:
					outgoingIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
			if isCollidingAtMid != isCollidingAtEnd:
				if isCollidingAtMid and not isCollidingAtEnd:
					outgoingIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
			if isCollidingAtStart == isCollidingAtMid and isCollidingAtMid == isCollidingAtEnd:
				if firstHalfBb.intersects(mapEdge):
					collisionIntervals.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				if secondHalfBb.intersects(mapEdge):
					collisionIntervals.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
		return (ingoingIntervals, outgoingIntervals)

	def _findIntermediateComponentEvents(self, previousSensor: SensingRegion, currentSensor: SensingRegion, envMap: Map) -> List[Polygon]:
		"""
			Given the original configuration of the FOV (`previousSensor`) and the final configuration (`currentSensor`)
			find all the times where there is a shadow component event.

			That is, as the FOV moves towards its final configuration save the intermediate configurations for which there is a topological change (FOV hitting a "gap").
			#### Returns
			A list of intermediate between which there is at most one component event.
		"""
		centerOfRotation = previousSensor.polygon.exterior.coords[3]
		transformation = Geometry.getAffineTransformation(previousSensor.polygon, currentSensor.polygon, centerOfRotation)
		previousCollidingEdges = self._getCollidingEdgesByEdge(previousSensor, envMap)
		currentCollidingEdges = self._getCollidingEdgesByEdge(currentSensor, envMap)
		intermediateCollisions = self._findCollisionsWithExtendedBb(previousSensor, transformation, centerOfRotation, envMap)
		# Remove the edges that we are sure are intersecting
		for id in previousCollidingEdges:
			for l in previousCollidingEdges[id]:
				if id in intermediateCollisions: intermediateCollisions[id].remove(l)
		for id in currentCollidingEdges:
			for l in currentCollidingEdges[id]:
				if id in intermediateCollisions: intermediateCollisions[id].remove(l)
		collisionIntervals = self._initCollisionIntervals(previousSensor, intermediateCollisions)

		while len(collisionIntervals) > 0:
			(ingoingIntervals, outgoingIntervals) = self._findEventIntervalsForCollisions(previousSensor, collisionIntervals, transformation, centerOfRotation)
		# Each interval is represented as a tuple: (sensorEdge, mapEdge, float, float)
		# The first float is the interval start and the second one is interval end times.
		intervals = self._initEdgeIntervals(previousSensor, transformation, centerOfRotation, previousCollidingEdges, inverse=False)
		haveOverlap = intervals + self.outgoingIntervals
		dontHaveOverlap = []
		eventCandidates = []
		while len(haveOverlap) > 0:
			i = 0
			while i < len(haveOverlap):
				(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
				intervalMid = (intervalStart + intervalEnd) / 2
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalMid)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalMid, intervalEnd)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.polygon, centerOfRotation)
				eventCandidates.append((p, interval[3], "outgoing", interval[1]))
		intervals = self._initEdgeIntervals(currentSensor, transformation, centerOfRotation, currentCollidingEdges, inverse=True)
		haveOverlap = intervals + self.ingoingIntervals
		dontHaveOverlap = []
		while len(haveOverlap) > 0:
			i = 0
			while i < len(haveOverlap):
				(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
				intervalMid = (intervalStart + intervalEnd) / 2
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalStart, intervalMid)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				collsionCheckResults = self._checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, centerOfRotation, intervalMid, intervalEnd)
				if collsionCheckResults[0] != collsionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
			(haveOverlap, dontHaveOverlap) = self._splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, previousSensor.polygon, centerOfRotation)
				eventCandidates.append((p, interval[3], "ingoing", interval[1]))
		eventCandidates.sort(key=lambda e: e[1])
		return eventCandidates

	def _build(self, graphs: List[ConnectivityGraph], startInd = 0, endInd = None):
		if endInd is None: endInd = len(graphs)

		self.nodesByLayer = []
		self.eventsByLayer = []
		previousLayer = None
		startTime = time.time()
		for i in range(startInd, endInd):
			currentLayer = graphs[i]
			if previousLayer is None:
				gDense: ConnectivityGraph = currentLayer.condense()
				previousLayer = currentLayer
			else:
				for sensorId in previousLayer.fov.sensors:
					self.eventsByLayer.append(self._findIntermediateComponentEvents(
						previousLayer.fov.sensors[sensorId].region,
						currentLayer.fovUnion.getEquivalentSensorById(sensorId).region,
						previousLayer.map))
			return

			layerIndex = len(self.nodesByLayer)
			self.nodesByLayer.append([])
			self._timeStamps.append(currentLayer.timestamp)
			print ("Chaining for %.1f" % currentLayer.timestamp)
			gDense = currentLayer.condense()
			for n in gDense.nodes:
				newNode = GraphAlgorithms._getTimedNodeName(n, currentLayer.timestamp)
				self.add_node(newNode)
				n = n if n in currentLayer.nodes else gDense.nodes[n]["mappedName"]
				GraphAlgorithms.cloneNodeProps(currentLayer.nodes[n] if not n.startswith("sym") else gDense.nodes[n], self.nodes[newNode])
				self.nodes[newNode]["polygonName"] = n # This is lazy way to obtain the name of the polygon without doing string split etc.
				self.nodes[newNode]["cluster"] = currentLayer.nodeClusters[currentLayer.nodeToClusterMap[n]]
				self.nodesByLayer[layerIndex].append(newNode)
			for e in gDense.edges:
				self.add_edge(GraphAlgorithms._getTimedNodeName(e[0], currentLayer.timestamp), GraphAlgorithms._getTimedNodeName(e[1], currentLayer.timestamp), isTemporal=False)
			if layerIndex == 0:
				previousLayer = currentLayer
				continue
			# CREATE TEMPORAL EDGES
			for n1 in self.nodesByLayer[layerIndex - 1]:
				if GraphAlgorithms.isBeamNode(n1): continue
				for n2 in self.nodesByLayer[layerIndex]:
					if GraphAlgorithms.isBeamNode(n2): continue
					if self.nodes[n1]["type"] == "sensor" and self.nodes[n2]["type"] != "sensor": continue
					if self.nodes[n1]["type"] == "sensor" and self.nodes[n2]["type"] == "sensor":
						if self.nodes[n1]["polygonName"] != self.nodes[n2]["polygonName"]: continue
						self._addTemporalEdge(n1, n2)
						break
					if not Geometry.polygonAndPolygonIntersect(self.nodes[n1]["cluster"].polygon, self.nodes[n2]["cluster"].polygon): continue
					intersection = Geometry.intersect(self.nodes[n1]["cluster"].polygon, self.nodes[n2]["cluster"].polygon)[0] # We are sure there will be one intersection
					(pXs, pYs) = intersection.exterior.coords.xy
					connectedTemporally = True
					for sensorKey in currentLayer.fov.sensors:
						currentLayerSensor = currentLayer.fov.sensors[sensorKey]
						if not Geometry.haveOverlappingEdge(currentLayerSensor.region.polygon, intersection): continue
						for i in range(-1, len(currentLayerSensor._originalCoords)):
							(x1, y1) = currentLayerSensor._originalCoords[i]
							(x2, y2) = currentLayerSensor._originalCoords[i + 1]
							(x3, y3) = fovCoordsMap[(x1, y1)]
							(x4, y4) = fovCoordsMap[(x2, y2)]
							for j in range(len(pXs)):
								if self._testConnectivity(x1, y1, x2, y2, x3, y3, x4, y4, pXs[j], pYs[j]):
									# This means the instersection area was swept away by a sensor
									connectedTemporally = False
								if not connectedTemporally: break
							if not connectedTemporally: break
						if not connectedTemporally: continue
						self._addTemporalEdge(n1, n2)
					self._oldAlg(n1 ,n2)
			previousLayer = currentLayer
		endTime = time.time()
		print("%.3f s" % (endTime - startTime))

	def _oldAlg(self, n1, n2):
		shouldConnect = False
		if self.nodes[n1]["polygonName"] == self.nodes[n2]["polygonName"]: shouldConnect = True
		if not shouldConnect and self.nodes[n1]["polygonName"] in self.nodes[n1]["cluster"].nodes: shouldConnect = True
		if not shouldConnect: return
		self._addTemporalEdge(n1, n2)

	def getTemporalNeighbors(self, node: str, nodeLayerIndex: int = 0):
		timedName = GraphAlgorithms._getTimedNodeName(node, self._timeStamps[nodeLayerIndex])
		neighbors = set()
		for outEdge in self.out_edges(timedName):
			neighbor = outEdge[1]
			if not self.get_edge_data(outEdge[0], outEdge[1])["isTemporal"]: continue
			neighbors.add(neighbor)
		return neighbors

	def displayGraph(self, displayGeomGraph, displaySpringGraph):
		if displayGeomGraph:
			self._fig = GraphAlgorithms.displayGeometricGraph(self, self._regionNodes, self._beamNodes)
		else:
			self._fig = GraphAlgorithms.displaySpringGraph(self, self._regionNodes, self._beamNodes)

	def killDisplayedGraph(self):
		if self._fig:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
