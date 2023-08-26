from typing import Dict, List, Tuple, Union

from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, MultiPolygon, Polygon


class ContinuousTimeCollisionDetection:
	"""
		This class Contains functions related to Continuous-time Collision Detection.
		© Reza Teshnizi 2018-2023
	"""

	CollisionEvent = Tuple[Polygon, float, str, LineString]
	"""### Collision Event
	```
	(Polygon, timeOfEvent, "ingoing" | "outgoing", LineString)
	```
	Wherein `LineString` is the edge of the is colliding with `Polygon` after timeOfEvent.
	"""

	RegionUnionCollisions = Dict[str, Dict[str, List[LineString]]]
	"""
	Represents all the collisions between two polygons.
	### Type
	```
	{"regionName": {"edgeId": {L1, L2, ...}}}
	Dict[tuple, Dict[str, List[LineString]]]
	```
	"""

	CollisionInterval = Tuple[LineString, LineString, float, float]
	"""## Collision Interval

	A collision interval is a represents an interval of time in which two edges.
	The two floats are the two closed bounds of the interval of time when this event happens.
	That is, Given a tuple `(L1, L2, T1, T2)`, we get the interval of event: `[ T1 * fov1.t, T2 * fov2.t ]`

	"""

	@classmethod
	def __expandVertObbWithAngularVelocity(cls, coords: Geometry.Coords, angle: float, centerOfRotation: Geometry.Coords, expandAway = True):
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	@classmethod
	def __getCollidingEdgesBySensor(cls, sensorUnion: FieldOfView, polygon: Union[Polygon, MultiPolygon]) -> RegionUnionCollisions:
		collisionData: RegionUnionCollisions = {}
		for sensorName in sensorUnion:
			for edgeName in sensorUnion[sensorName].__edges:
				collisionData[sensorName] = cls.findCollisionsWithExtendedBb(polygon)

	@classmethod
	def __estimateLocationWithSlerp(cls, past: FieldOfView, ratio: float) -> Union[Polygon, MultiPolygon]:
		"""
			#### Input

			`past` a configuration of the fov (one-to-one mapping of the sensors) in the past,
			`ratio` a number between `0` (`self`) and `1` (`past`)

			#### Returns
			a `Polygon` or a `MultiPolygon` representing the state of the fov at time ratio.
		"""
		polygons = []
		for sensorName in self.__sensors:
			sensor = self[sensorName]
			pastSensor = past[sensorName]
			centerOfRotation = (self.pose.x, self.pose.y)
			transformation = Geometry.getAffineTransformation(sensor.region.polygon, pastSensor.region.polygon, centerOfRotation)
			intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, ratio)
			polygons.append(Geometry.applyMatrixTransformToPolygon(intermediateTransform, self.polygon, centerOfRotation))
		polygons = Geometry.union(polygons)
		return polygons

	@classmethod
	def __initCollisionIntervals(cls, sensor: SensorRegion, intermediateCollisions: RegionUnionCollisions) -> CollisionInterval:
		intervals = []
		for sensorEdgeId in intermediateCollisions:
			sensorEdge = sensor.__edges[sensorEdgeId]
			mapEdges = intermediateCollisions[sensorEdgeId]
			if (len(mapEdges) == 0): continue
			for mapEdge in mapEdges:
				intervals.append((sensorEdge, mapEdge, 0, 1))
		return intervals

	@classmethod
	def getCollidingEdges(cls, region: PolygonalRegion, polygon: Union[Polygon, MultiPolygon]) -> PolygonalRegion.Edges:
		"""## Get Colliding Edges By Edge
		Just like the name says.

		Parameters
		----------
		polygon : Union[Polygon, MultiPolygon]
			The polygon against whom we are checking collisions.

		Returns
		-------
		Dict[str, Set[LineString]]
			Map from the string of edgeName of the PolygonalRegion to edge of the given polygon.
		"""
		collisionData = {}
		for edgeName in region.edges:
			collisionData[edgeName] = Geometry.getAllIntersectingEdgesWithLine(cls.edges[edgeName], polygon)
		return collisionData

	@classmethod
	def getLineSegmentExpandedBb(cls, transformation: AffineTransform, lineSeg: LineString, angle: float, centerOfRotation: Geometry.Coords) -> Polygon:
		""" Gets a tight bounding box for a line segment that is moving with a constant angular velocity. """
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg, centerOfRotation)
		polygons = []
		for j in range(16):
			v1 = cls.__expandVertObbWithAngularVelocity(lineSeg.coords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = cls.__expandVertObbWithAngularVelocity(finalConfig.coords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = cls.__expandVertObbWithAngularVelocity(finalConfig.coords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = cls.__expandVertObbWithAngularVelocity(lineSeg.coords[1], angle, centerOfRotation, j & 8 != 0)
			p = Polygon([v1, v2, v3, v4])
			polygons.append(p)
		expandedObb = Geometry.union(polygons)
		expandedObb = expandedObb.convex_hull
		return expandedObb

	@classmethod
	def getCollidingEdgesWithExtendedBb(cls, past: PolygonalRegion, now: PolygonalRegion, polygon: Union[Polygon, MultiPolygon]) -> Dict[str, List[LineString]]:
		"""
			#### Returns
				For each edge of a moving region, given as `past` an `now`,
				it returns all the colliding edges of the `polygon` that intersect the expanded bounding box of that edge.
		"""
		centerOfRotation = (now.centerOfRotation.x, now.centerOfRotation.y)
		transformation = Geometry.getAffineTransformation(Polygon(past.envelope), Polygon(now.envelope), centerOfRotation)
		collisionData: Dict[str, List[LineString]] = {}
		angle = abs(transformation.rotation)
		for sensorEdgeId in now.edges:
			collisionData[sensorEdgeId] = []
			edge = now.edges[sensorEdgeId]
			boundingBox = cls.getLineSegmentExpandedBb(transformation, edge, angle, centerOfRotation)
			verts = polygon.exterior.coords
			for v1, v2 in zip(verts, verts[1:]):
				e = LineString([v1, v2])
				if boundingBox.intersects(e):
					collisionData[sensorEdgeId].append(e)
			# FIXME: Iterate over interior rings as well.
		return collisionData

	@classmethod
	def estimateIntermediateCollisionsWithPolygon(cls, pastSensors: FieldOfView, nowSensors: FieldOfView, polygon: Union[Polygon, MultiPolygon]) -> CollisionEvent:
		"""## Estimate Intermediate Collisions With Polygon

			Given past and the current configuration of a sensors,
			find all the times when there is a shadow component event.
			That is, as the `past` FOV moves towards its current configuration (`self`),
			save the intermediate configurations for which there is a topological change.
			This function assumes each FOV has gone through an affine transformation between two consecutive time frames.

			Parameters
			----------
			past : Fov
				The `past` configuration of the FOV
			polygon : Polygon | List[Polygon] | MultiPolygon
				The second polygon with whom we are interested in checking intersection of edges.

			Returns
			-------
			Events
				A list of intervals in each of which there is at most one component event.
			"""

		collisionData: RegionUnionCollisions = {}
		# Begin by collecting the edges are that are in contact at the beginning and at the end of the motion.
		for sensorId in pastSensors:
			collisionData[sensorId] = cls.getCollidingEdgesWithExtendedBb(pastSensors[sensorId], nowSensors[sensorId], polygon)
			pastCollidingEdges = cls.getCollidingEdges(pastSensors[sensorId], polygon)
			nowCollidingEdges = cls.getCollidingEdges(nowSensors[sensorId], polygon)
			transformation = Geometry.getAffineTransformation(pastSensors[sensorId].envelope, currentSensor.region.polygon, centerOfRotation)
			intermediateCollisions = self._findCollisionsWithExtendedBb(previousSensor.region, transformation, centerOfRotation, envMap.polygon)
			# Remove the edges that we are sure are intersecting
			for id in pastCollidingEdges:
				for l in pastCollidingEdges[id]:
					if id in intermediateCollisions: intermediateCollisions[id].remove(l)
			for id in nowCollidingEdges:
				for l in nowCollidingEdges[id]:
					if id in intermediateCollisions: intermediateCollisions[id].remove(l)
			collisionIntervals = self._initCollisionIntervals(previousSensor.region, intermediateCollisions)
			# [self.lines.append(e[1]) for e in collisionIntervals]

			(ingoingIntervals, outgoingIntervals) = ([], [])
			while len(collisionIntervals) > 0:
				self._findEventIntervalsForCollisions(previousSensor.region, collisionIntervals, transformation, centerOfRotation, ingoingIntervals, outgoingIntervals)
			# Each interval is represented as a tuple: (sensorEdge, mapEdge, float, float)
			# The first float is the interval start and the second one is interval end times.
			intervals = self._initEdgeIntervals(previousSensor.region, transformation, centerOfRotation, pastCollidingEdges, inverse=False)
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
			intervals = self._initEdgeIntervals(currentSensor.region, transformation, centerOfRotation, nowCollidingEdges, inverse=True)
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
