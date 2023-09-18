from typing import Dict, List, Set, Tuple, Union

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose


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

	RegionCollisions = Dict[str, List[LineString]]
	"""
	Represents all the collisions a Region and a Polygon.
	### Type
	```
	{"edgeId": {L1, L2, ...}}
	Dict[str, List[LineString]]
	```
	wherein, `edgeId` is the id of the region edge and `Li` is the edge of the other.
	"""

	RegularRegionCollisions = Dict[str, RegionCollisions]
	"""
	Represents all the collisions between a regular region and a polygon.
	### Type
	```
	{"subRegionName": {"edgeId": {L1, L2, ...}}}
	Dict[str, Dict[str, List[LineString]]]
	```
	"""

	CollisionInterval = Tuple[LineString, LineString, float, float]
	"""## Collision Interval

	A collision interval is a represents an interval of time in which two edges collide.
	A tuple `(L1, L2, T1, T2)`, the two floats are the bounds of the interval of time when this event happens: `[T1, T2)`.
	To construct, the absolute values of time we get the interval of event: `[ T1 * fov1.t, T2 * fov2.t ]`
	"""

	MIN_TIME_DELTA = 1
	"""
	### A Core Assumption:
	We expect the updates to be at least as fast as `MIN_TIME_DELTA` ns.
	"""

	@classmethod
	def __estimateLocationWithSlerp(cls, past: RegularAffineRegion, now: RegularAffineRegion, ratio: float) -> Union[Polygon, MultiPolygon]:
		"""
			#### Input

			`past` a configuration of the fov (one-to-one mapping of the sensors) in the past,
			`ratio` a number between `0` (`cls`) and `1` (`past`)

			#### Returns
			a `Polygon` or a `MultiPolygon` representing the state of the fov at time ratio.
		"""
		polygons = []
		for movingRegionId in past:
			sensor = now[movingRegionId]
			pastSensor = past[movingRegionId]
			centerOfRotation = (cls.pose.x, cls.pose.y)
			transformation = Geometry.getAffineTransformation(sensor.region.polygon, pastSensor.region.polygon, centerOfRotation)
			intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, ratio)
			polygons.append(Geometry.applyMatrixTransformToPolygon(intermediateTransform, cls.polygon, centerOfRotation))
		polygons = Geometry.union(polygons)
		return polygons

	@classmethod
	def __checkIntervalsForOverlap(cls, interval1: Tuple[float, float], interval2: Tuple[float, float]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	@classmethod
	def __edgesHaveACommonVertex(cls, l1: LineString, l2: LineString) -> bool:
		l1Verts = l1.coords
		l2Verts = l2.coords
		for v1 in l1Verts:
			for v2 in l2Verts:
				if Geometry.coordsAreAlmostEqual(v1, v2): return True
		return False

	@classmethod
	def __splitIntervalsListForOverlap(cls, intervals: List[Tuple[LineString, LineString, float, float]]) -> Tuple[List[Tuple[LineString, LineString, float, float]], List[Tuple[LineString, LineString, float, float]]]:
		haveOverlap = []
		dontHaveOverlap = []
		while len(intervals) > 0:
			interval1 = intervals.pop()
			foundOverlap = False
			for interval2 in intervals:
				if interval1 == interval2: continue
				if cls.__checkIntervalsForOverlap(interval1[2:], interval2[2:]):
					if interval1[0] == interval2[0]:
						# If the sensor edges are the same
						if cls.__edgesHaveACommonVertex(interval1[1], interval2[1]):
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

	@classmethod
	def __expandVertObbWithAngularVelocity(cls, coord: Geometry.Coords, angle: float, centerOfRotation: Pose, expandAway = True) -> Tuple[float, float]:
		displacement = (coord[0] - centerOfRotation.x, coord[1] - centerOfRotation.y)
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coord[0] + vertExpansion[0], coord[1] + vertExpansion[1]) if expandAway else (coord[0] - vertExpansion[0], coord[1] - vertExpansion[1])
		return expanded

	@classmethod
	def __checkEdgeIntervalForCollision(cls, movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform, centerOfRotation: Pose, intervalStart: float, intervalEnd: float) -> Tuple[bool, bool]:
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		movingEdgeStartConfiguration = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge, centerOfRotation)
		movingEdgeEndConfiguration = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		return (movingEdgeStartConfiguration.intersects(staticEdge), movingEdgeEndConfiguration.intersects(staticEdge))

	@classmethod
	def __initEdgeIntervals(cls, movingRegion: AffineRegion, transformation: AffineTransform, collisionData: RegionCollisions, inverse: bool) -> List[CollisionInterval]:
		"""
			Initialize intervals for edges that are currently collisions with the sensor.
		"""
		intervals: List[cls.CollisionInterval] = []
		intStart = 1 if inverse else 0
		intEnd = 0 if inverse else 1
		for collidingEdgeId in collisionData:
			collidingEdges = collisionData[collidingEdgeId]
			if (len(collidingEdges) == 0): continue
			movingEdge = movingRegion.edges[collidingEdgeId]
			for collidingEdge in collidingEdges:
				collisionCheckResults = cls.__checkEdgeIntervalForCollision(movingEdge, collidingEdge, transformation, movingRegion.centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collisionCheckResults[0] == collisionCheckResults[1]: continue
				intervals.append((movingEdge, collidingEdge, 0, 1))
		return intervals

	@classmethod
	def __getLineSegmentExpandedBb(cls, lineSeg: LineString, transformation: AffineTransform, centerOfRotation: Pose) -> Polygon:
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg, centerOfRotation)
		polygons = []
		for j in range(16):
			v1 = cls.__expandVertObbWithAngularVelocity(lineSeg.coords[0], transformation.rotation, centerOfRotation, j & 1 != 0)
			v2 = cls.__expandVertObbWithAngularVelocity(finalConfig.coords[0], transformation.rotation, centerOfRotation, j & 2 != 0)
			v3 = cls.__expandVertObbWithAngularVelocity(finalConfig.coords[1], transformation.rotation, centerOfRotation, j & 4 != 0)
			v4 = cls.__expandVertObbWithAngularVelocity(lineSeg.coords[1], transformation.rotation, centerOfRotation, j & 8 != 0)
			p = Polygon([v1, v2, v3, v4])
			polygons.append(p)
		expandedObb = Geometry.union(polygons)
		expandedObb = expandedObb.convex_hull
		return expandedObb

	@classmethod
	def __checkBoundingBoxIntervalForCollision(cls, region: AffineRegion, movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform, centerOfRotation: Pose, intervalStart: float, intervalEnd: float) -> Tuple[Tuple[bool, bool, bool], Polygon, Polygon]:
		"""
			Returns
			-------
			`Tuple[Tuple[bool, bool, bool], Polygon, Polygon, LineString]`
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
		startSensor = Geometry.applyMatrixTransformToPolygon(startConfigTransformation, region.interior, centerOfRotation)
		midSensor = Geometry.applyMatrixTransformToPolygon(midConfigTransformation, region.interior, centerOfRotation)
		endSensor = Geometry.applyMatrixTransformToPolygon(endConfigTransformation, region.interior, centerOfRotation)
		firstHalfTransformation = Geometry.getAffineTransformation(startSensor, midSensor, centerOfRotation)
		secondHalfTransformation = Geometry.getAffineTransformation(midSensor, endSensor, centerOfRotation)
		firstHalfBb = cls.__getLineSegmentExpandedBb(edgeAtStart, firstHalfTransformation, centerOfRotation)
		secondHalfBb = cls.__getLineSegmentExpandedBb(edgeAtMid, secondHalfTransformation, centerOfRotation)
		return ((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb)

	@classmethod
	def __estimateEventIntervalsForCollisions(cls, movingRegion: AffineRegion, collisionIntervals: List[CollisionInterval], transformation: AffineTransform, ingoingIntervals: List[CollisionInterval], outgoingIntervals: List[CollisionInterval]):
		i = 0
		while i < len(collisionIntervals):
			(sensorEdge, mapEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= cls.MIN_TIME_DELTA: continue # Minimum time interval assumption

			((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb) = cls.__checkBoundingBoxIntervalForCollision(movingRegion, sensorEdge, mapEdge, transformation, movingRegion.centerOfRotation, intervalStart, intervalEnd)
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

	@classmethod
	def __removeExistingCollisionsFromIntermediateCollisions(cls, existingCollisions: RegionCollisions, intermediateCollisions: RegionCollisions) -> RegionCollisions:
		"""
		Remove the edges that we are sure are intersecting.

		Parameters
		----------
		existingCollisions : RegionCollisions
			Existing collisions.
		intermediateCollisions : RegionCollisions
			Collisions that happen within a certain bounded time interval.
		"""
		for movingEdgeId in existingCollisions:
			for collidingEdge in existingCollisions[movingEdgeId]:
				if movingEdgeId in intermediateCollisions and collidingEdge in intermediateCollisions[movingEdgeId]:
					intermediateCollisions[movingEdgeId].remove(collidingEdge)
		return intermediateCollisions

	@classmethod
	def getCollidingEdges(cls, region: PolygonalRegion, polygon: Union[Polygon, MultiPolygon]) -> RegionCollisions:
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
		collisionData: cls.RegionCollisions = {}
		for edgeName in region.edges:
			collisionData[edgeName] = Geometry.getAllIntersectingEdgesWithLine(region.edges[edgeName], polygon)
		return collisionData

	@classmethod
	def getLineSegmentExpandedBb(cls, transformation: AffineTransform, lineSeg: LineString, angle: float, centerOfRotation: Pose) -> Polygon:
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
	def getCollidingEdgesWithExtendedBb(cls, past: AffineRegion, now: AffineRegion, polygon: Union[Polygon, MultiPolygon]) -> RegionCollisions:
		"""
			#### Returns
				For each edge of a moving region, given as `past` an `now`,
				it returns all the colliding edges of the `polygon` that intersect the expanded bounding box of that edge.
		"""
		transformation = Geometry.getAffineTransformation(past.envelope, now.envelope, past.centerOfRotation)
		collisionData: cls.RegionCollisions = {}
		angle: float = abs(transformation.rotation)
		for sensorEdgeId in now.edges:
			collisionData[sensorEdgeId] = []
			edge = now.edges[sensorEdgeId]
			boundingBox = cls.getLineSegmentExpandedBb(transformation, edge, angle, past.centerOfRotation)
			verts = polygon.exterior.coords
			for v1, v2 in zip(verts, verts[1:]):
				e = LineString([v1, v2])
				if boundingBox.intersects(e):
					collisionData[sensorEdgeId].append(e)
			# FIXME: Iterate over interior rings as well.
		return collisionData

	@classmethod
	def estimateIntermediateCollisionsWithPolygon(cls, pastSensors: FieldOfView, nowSensors: FieldOfView, polygon: Union[Polygon, MultiPolygon]) -> CollisionEvent:
		"""## Estimate Intermediate Collisions With A Fixed Polygon

			Given past and the current configuration of a sensors,
			find all the times when there is a shadow component event.
			That is, as the `past` FOV moves towards its current configuration (`cls`),
			save the intermediate configurations for which there is a topological change.
			This function assumes each FOV has gone through an affine transformation between two consecutive time frames.

			Parameters
			----------
			past : `Fov`
				The `past` configuration of the FOV
			polygon : `Polygon | List[Polygon] | MultiPolygon`
				The second polygon with whom we are interested in checking intersection of edges.

			Returns
			-------
			Events
				A list of intervals in each of which there is at most one component event.
			"""

		collisionData: cls.RegularRegionCollisions = {}
		# Begin by collecting the edges are that are in contact at the beginning and at the end of the motion.
		for sensorId in pastSensors:
			RosUtils.Logger().info("Estimating intermediate collisions between %s and the moving region between %s." % (repr(polygon), sensorId))
			collisionData[sensorId] = cls.getCollidingEdgesWithExtendedBb(pastSensors[sensorId], nowSensors[sensorId], polygon)
			pastCollidingEdges = cls.getCollidingEdges(pastSensors[sensorId], polygon)
			RosUtils.Logger().info("Past colliding edges:\t%s" % repr(pastCollidingEdges))
			nowCollidingEdges = cls.getCollidingEdges(nowSensors[sensorId], polygon)
			RosUtils.Logger().info("Now colliding edges:\t%s" % repr(nowCollidingEdges))
			transformation = Geometry.getAffineTransformation(pastSensors[sensorId].envelope, nowSensors[sensorId].envelope, pastSensors[sensorId].centerOfRotation)
			intermediateCollisions = cls.getCollidingEdgesWithExtendedBb(pastSensors[sensorId], nowSensors[sensorId], polygon)
			intermediateCollisions = cls.__removeExistingCollisionsFromIntermediateCollisions(pastCollidingEdges, intermediateCollisions)
			intermediateCollisions = cls.__removeExistingCollisionsFromIntermediateCollisions(nowCollidingEdges, intermediateCollisions)
			RosUtils.Logger().info("IntermediateCollisions:\t%s" % repr(intermediateCollisions))

			ingoingIntervals: List[cls.CollisionInterval] = []
			outgoingIntervals: List[cls.CollisionInterval] = []
			collisionIntervals = cls.__initEdgeIntervals(pastSensors[sensorId], transformation, pastCollidingEdges, inverse=False)
			while len(collisionIntervals) > 0:
				RosUtils.Logger().info("Current Intervals:\t%s" % repr(collisionIntervals))
				cls.__estimateEventIntervalsForCollisions(pastSensors[sensorId], collisionIntervals, transformation, ingoingIntervals, outgoingIntervals)
			# Each interval is represented as a tuple: (sensorEdge, mapEdge, float, float)
			# The first float is the interval start and the second one is interval end times.
			intervals = cls.__initEdgeIntervals(pastSensors[sensorId], transformation, pastCollidingEdges, inverse=False)
			haveOverlap = intervals + outgoingIntervals
			dontHaveOverlap: List[cls.CollisionInterval] = []
			eventCandidates: List[cls.CollisionInterval] = []
			i = 0
			while len(haveOverlap) > 0 and i < len(haveOverlap):
				(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
				intervalMid = (intervalStart + intervalEnd) / 2
				collisionCheckResults = cls.__checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, pastSensors[sensorId].centerOfRotation, intervalStart, intervalMid)
				if collisionCheckResults[0] != collisionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				collisionCheckResults = cls.__checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, pastSensors[sensorId].centerOfRotation, intervalMid, intervalEnd)
				if collisionCheckResults[0] != collisionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
				(haveOverlap, dontHaveOverlap) = cls.__splitIntervalsListForOverlap(haveOverlap)
				for interval in dontHaveOverlap:
					intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
					p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, pastSensors[sensorId].interior, pastSensors[sensorId].centerOfRotation)
					eventCandidates.append((p, interval[3], "outgoing", interval[1]))
			intervals = cls.__initEdgeIntervals(nowSensors[sensorId], transformation, nowCollidingEdges, inverse=True)
			haveOverlap = intervals + ingoingIntervals
			dontHaveOverlap: List[cls.CollisionInterval] = []
			i = 0
			while len(haveOverlap) > 0 and i < len(haveOverlap):
				(sensorEdge, mapEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
				intervalMid = (intervalStart + intervalEnd) / 2
				collisionCheckResults = cls.__checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, pastSensors[sensorId].centerOfRotation, intervalStart, intervalMid)
				if collisionCheckResults[0] != collisionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalStart, intervalMid))
					i += 1
				collisionCheckResults = cls.__checkEdgeIntervalForCollision(sensorEdge, mapEdge, transformation, pastSensors[sensorId].centerOfRotation, intervalMid, intervalEnd)
				if collisionCheckResults[0] != collisionCheckResults[1]:
					haveOverlap.insert(i, (sensorEdge, mapEdge, intervalMid, intervalEnd))
					i += 1
				(haveOverlap, dontHaveOverlap) = cls._splitIntervalsListForOverlap(haveOverlap)
				for interval in dontHaveOverlap:
					intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
					p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, pastSensors[sensorId].interior, pastSensors[sensorId].centerOfRotation)
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
