from typing import Dict, List, Literal, Tuple, Union

from visualization_msgs.msg import MarkerArray

from rt_bi_core.Eventifier.FieldOfView import FieldOfView
from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.RegularAffineRegion import RegularAffineRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.Ros import Logger, Publisher
from rt_bi_utils.RViz import Color, KnownColors, RViz


class ContinuousTimeCollisionDetection:
	"""
		This class contains functions related to Continuous-time Collision Detection.
		© Reza Teshnizi 2018-2023
	"""

	CollisionEvent = Tuple[Polygon, int, Pose, float, Literal["ingoing", "outgoing"], LineString]
	"""### Collision Event
	```
	(Polygon, idNum, centerOfRotation, timeOfEvent, "ingoing" | "outgoing", collidingEdge)
	```
	Wherein `collidingEdge` is the edge of the `Polygon` that is colliding at some point after `0 ≤ timeOfEvent≤ 1`.
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

	__rvizPublisher: Union[Publisher, None] = None

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
		return (Geometry.intersects(movingEdgeStartConfiguration,staticEdge), Geometry.intersects(movingEdgeEndConfiguration, staticEdge))

	@classmethod
	def __initEdgeIntervals(cls, movingRegion: AffineRegion, transformation: AffineTransform, existingCollisions: RegionCollisions, inverse: bool) -> List[CollisionInterval]:
		"""
			Initialize intervals for edges that are currently collisions with the sensor.
		"""
		intervals: List[cls.CollisionInterval] = []
		intStart = 1 if inverse else 0
		intEnd = 0 if inverse else 1
		for movingEdgeId in existingCollisions:
			staticEdges = existingCollisions[movingEdgeId]
			if (len(staticEdges) == 0): continue
			movingEdge = movingRegion.edges[movingEdgeId]
			for staticEdge in staticEdges:
				(collidingAtTheBeginning, collidingAtTheEnd) = cls.__checkEdgeIntervalForCollision(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalStart=intStart, intervalEnd=intEnd)
				if collidingAtTheBeginning == collidingAtTheEnd: continue
				intervals.append((movingEdge, staticEdge, 0, 1))
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
		isCollidingAtStart = Geometry.intersects(edgeAtStart, staticEdge) is not None
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		edgeAtEnd = Geometry.applyMatrixTransformToLineString(endConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtEnd = Geometry.intersects(edgeAtEnd, staticEdge) is not None
		intervalMid = (intervalStart + intervalEnd) / 2
		midConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalMid)
		edgeAtMid = Geometry.applyMatrixTransformToLineString(midConfigTransformation, movingEdge, centerOfRotation)
		isCollidingAtMid = Geometry.intersects(edgeAtMid, staticEdge) is not None
		startSensor = Geometry.applyMatrixTransformToPolygon(startConfigTransformation, region.interior, centerOfRotation)
		midSensor = Geometry.applyMatrixTransformToPolygon(midConfigTransformation, region.interior, centerOfRotation)
		endSensor = Geometry.applyMatrixTransformToPolygon(endConfigTransformation, region.interior, centerOfRotation)
		firstHalfTransformation = Geometry.getAffineTransformation(startSensor, midSensor, centerOfRotation)
		secondHalfTransformation = Geometry.getAffineTransformation(midSensor, endSensor, centerOfRotation)
		firstHalfBb = cls.__getLineSegmentExpandedBb(edgeAtStart, firstHalfTransformation, centerOfRotation)
		secondHalfBb = cls.__getLineSegmentExpandedBb(edgeAtMid, secondHalfTransformation, centerOfRotation)
		return ((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb)

	@classmethod
	def __estimateCollisionIntervals(cls, movingRegion: AffineRegion, transformation: AffineTransform, collisionIntervals: List[CollisionInterval]) -> Tuple[List[CollisionInterval], List[CollisionInterval]]:
		"""### Estimate Collision Intervals

		Parameters
		----------
		movingRegion : `AffineRegion`
			The moving region.
		transformation : `AffineTransform`
			The transformation matrix describing its movement.
		collisionIntervals : `List[CollisionInterval]`
			The list of existing collision intervals to further refine for estimation.

		Returns
		-------
		`Tuple[List[CollisionInterval], List[CollisionInterval]]`
			`(ingoingIntervals, outgoingIntervals)`
			The collisions happening for the ingoing and outgoing transformation, respectively.
		"""
		ingoingIntervals: List[cls.CollisionInterval] = []
		outgoingIntervals: List[cls.CollisionInterval] = []
		i = 0
		while i < len(collisionIntervals):
			(movingEdge, staticEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= cls.MIN_TIME_DELTA: continue # Minimum time interval assumption

			((isCollidingAtStart, isCollidingAtMid, isCollidingAtEnd), firstHalfBb, secondHalfBb) = cls.__checkBoundingBoxIntervalForCollision(movingRegion, movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalStart, intervalEnd)
			intervalMid = (intervalStart + intervalEnd) / 2
			if isCollidingAtStart != isCollidingAtMid:
				if isCollidingAtStart and not isCollidingAtMid:
					outgoingIntervals.append((movingEdge, staticEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.append((movingEdge, staticEdge, intervalStart, intervalMid))
			if isCollidingAtMid != isCollidingAtEnd:
				if isCollidingAtMid and not isCollidingAtEnd:
					outgoingIntervals.append((movingEdge, staticEdge, intervalStart, intervalMid))
				else:
					ingoingIntervals.append((movingEdge, staticEdge, intervalStart, intervalMid))
			if isCollidingAtStart == isCollidingAtMid and isCollidingAtMid == isCollidingAtEnd:
				if Geometry.intersects(firstHalfBb, staticEdge):
					collisionIntervals.insert(i, (movingEdge, staticEdge, intervalStart, intervalMid))
					i += 1
				if Geometry.intersects(secondHalfBb, staticEdge):
					collisionIntervals.insert(i, (movingEdge, staticEdge, intervalMid, intervalEnd))
					i += 1
		return (ingoingIntervals, outgoingIntervals)

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
	def __refineIntervals(cls, movingRegion: AffineRegion, transformation: AffineTransform, startingCollisions: RegionCollisions, estimatedCollision: List[CollisionInterval], eventCandidates: List[CollisionEvent], inverse: bool) -> List[CollisionEvent]:
		"""## Refine Intervals
		Performs a binary search until there are no more overlap among the intervals of time in which
		a collision would occur between an edge of the movingRegion and the given `collidingEdge`

		Parameters
		----------
		movingRegion : `AffineRegion`
			_description_
		transformation : `AffineTransform`
			_description_
		startingCollisions : `RegionCollisions`
			_description_
		estimatedCollision : `List[CollisionInterval]`
			_description_
		eventCandidates : `List[CollisionEvent]`
			_description_
		inverse : `bool`
			_description_

		Returns
		-------
		`List[CollisionEvent]`
			_description_
		"""
		# Logger().info("Refining Collision, %s pass:\t%s" % ("ingoing" if inverse else "outgoing", repr(estimatedCollision)))
		intervals = cls.__initEdgeIntervals(movingRegion, transformation, startingCollisions, inverse)
		haveOverlap = intervals + estimatedCollision
		dontHaveOverlap: List[cls.CollisionInterval] = []
		i = 0
		while len(haveOverlap) > 0 and i < len(haveOverlap):
			(movingEdge, staticEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
			cls.__renderLineString([movingEdge], KnownColors.PURPLE, 3)
			cls.__renderLineString([staticEdge], KnownColors.GREEN, 3)
			intervalMid = (intervalStart + intervalEnd) / 2
			collisionCheckResults = cls.__checkEdgeIntervalForCollision(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalStart, intervalMid)
			if collisionCheckResults[0] != collisionCheckResults[1]:
				haveOverlap.insert(i, (movingEdge, staticEdge, intervalStart, intervalMid))
				i += 1
			collisionCheckResults = cls.__checkEdgeIntervalForCollision(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalMid, intervalEnd)
			if collisionCheckResults[0] != collisionCheckResults[1]:
				haveOverlap.insert(i, (movingEdge, staticEdge, intervalMid, intervalEnd))
				i += 1
			(haveOverlap, dontHaveOverlap) = cls.__splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, movingRegion.interior, movingRegion.centerOfRotation)
				eventCandidates.append((p, movingRegion.idNum, movingRegion.centerOfRotation, interval[3], "ingoing" if inverse else "outgoing", interval[1]))
		return eventCandidates

	@classmethod
	def __getCollidingEdges(cls, region: PolygonalRegion, polygon: Union[Polygon, MultiPolygon]) -> RegionCollisions:
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
			collisionData[edgeName] = Geometry.getIntersectingEdges(region.edges[edgeName], polygon)
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
	def __getCollidingEdgesWithExtendedBb(cls, past: AffineRegion, now: AffineRegion, polygon: Union[Polygon, MultiPolygon]) -> RegionCollisions:
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
			if boundingBox.is_empty: continue
			verts = polygon.exterior.coords
			for v1, v2 in zip(verts, verts[1:]):
				e = LineString([v1, v2])
				if Geometry.intersects(boundingBox, e):
					collisionData[sensorEdgeId].append(e)
			# FIXME: Iterate over interior rings as well.
		return collisionData

	@classmethod
	def __renderLineString(cls, line: LineString, color: Color, renderWidth: float = 1.0) -> None:
		if cls.__rvizPublisher is None: return
		strId = repr(Geometry.lineStringId(line))
		marker = RViz.CreateLine(strId, Geometry.getGeometryCoords(line), color, renderWidth)
		msg = MarkerArray()
		msg.markers.append(marker)
		cls.__rvizPublisher.publish(msg)
		return

	@classmethod
	def estimateIntermediateCollisionsWithPolygon(cls, pastSensors: FieldOfView, nowSensors: FieldOfView, polygon: Union[Polygon, MultiPolygon], rvizPublisher: Union[Publisher, None]) -> List[CollisionEvent]:
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

		cls.__rvizPublisher = rvizPublisher
		collisionData: cls.RegularRegionCollisions = {}
		cls.__renderLineString(polygon.exterior, KnownColors.BLUE, 2)
		for sensorId in pastSensors:
			if sensorId not in nowSensors:
				continue
			Logger().info("Estimating intermediate collisions between %s and the moving region %s." % (repr(polygon), sensorId))

			collisionData[sensorId] = cls.__getCollidingEdgesWithExtendedBb(pastSensors[sensorId], nowSensors[sensorId], polygon)
			pastCollidingEdges = cls.__getCollidingEdges(pastSensors[sensorId], polygon)
			nowCollidingEdges = cls.__getCollidingEdges(nowSensors[sensorId], polygon)
			transformation = Geometry.getAffineTransformation(pastSensors[sensorId].envelope, nowSensors[sensorId].envelope, pastSensors[sensorId].centerOfRotation)

			# Following two liners are optimization lines and not necessary.
			# collisionData[sensorId] = cls.__removeExistingCollisionsFromIntermediateCollisions(pastCollidingEdges, collisionData[sensorId])
			# collisionData[sensorId] = cls.__removeExistingCollisionsFromIntermediateCollisions(nowCollidingEdges, collisionData[sensorId])
			if len(collisionData[sensorId]) == 0:
				Logger().info("Empty collision, skipping the refinement.")
				continue

			# Here we try to find the broadest set of collisions: any edge that collides with the OBB.
			# We also divide the collisions into two subgroups of ingoing and outgoing: from past to now and vice versa.
			ingoingIntervals: List[cls.CollisionInterval] = []
			outgoingIntervals: List[cls.CollisionInterval] = []
			collisionIntervals = cls.__initEdgeIntervals(pastSensors[sensorId], transformation, pastCollidingEdges, inverse=False)
			Logger().info("Initial Intervals:\t%s" % repr(collisionIntervals))
			while len(collisionIntervals) > 0:
				(ingoingIntervals, outgoingIntervals) = cls.__estimateCollisionIntervals(pastSensors[sensorId], transformation, collisionIntervals)
			eventCandidates = cls.__refineIntervals(pastSensors[sensorId], transformation, pastCollidingEdges, outgoingIntervals, [], inverse=False)
			eventCandidates = cls.__refineIntervals(nowSensors[sensorId], transformation, nowCollidingEdges, ingoingIntervals, eventCandidates, inverse=True)
			Logger().info("Total Events:\t%s" % repr(eventCandidates))
			# Sort events by time
			eventCandidates.sort(key=lambda e: e[3])
			# Remove duplicate times
			times = set()
			i = 0
			while i < len(eventCandidates):
				if eventCandidates[i][3] not in times:
					times.add(eventCandidates[i][1])
					i += 1
				else:
					eventCandidates.pop(i)
		return eventCandidates
