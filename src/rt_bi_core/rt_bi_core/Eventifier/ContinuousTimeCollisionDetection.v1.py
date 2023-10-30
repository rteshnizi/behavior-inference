from typing import Dict, List, Literal, Tuple, Union

from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, Point, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.Ros import Publisher


class ContinuousTimeCollisionDetection_V1:
	"""
		This class contains functions related to Continuous-time Collision Detection.
		© Reza Teshnizi 2018-2023
	"""

	CollisionEvent = Tuple[Polygon, int, Pose, float, Literal["made", "released"], LineString]
	"""### Collision Event
	```
	(Polygon, idNum, centerOfRotation, timeOfEvent, "made" | "released", collidingEdge)
	```
	Wherein:
		* `idNum` is the id of the edge in the source moving region.
		* `collidingEdge` is the edge of the `Polygon` that is colliding at some point after `0 ≤ timeOfEvent≤ 1`.
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
	def __checkCollisionAtTime(cls, movingEdge: LineString, staticEdge: LineString, transformation: AffineTransform, centerOfRotation: Pose, intervalTime: float) -> bool:
		transformationAtT = Geometry.getParameterizedAffineTransformation(transformation, intervalTime)
		movingEdgeAtT = Geometry.applyMatrixTransformToLineString(transformationAtT, movingEdge, centerOfRotation)
		return Geometry.intersects(movingEdgeAtT,staticEdge)

	@classmethod
	def __initEdgeIntervals(cls, movingRegion: DynamicRegion, transformation: AffineTransform, existingCollisions: RegionCollisions, inverse: bool) -> List[CollisionInterval]:
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
				(collidingAtTheBeginning, collidingAtTheEnd) =\
					(
						cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intStart),
						cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intEnd)
					)
				if collidingAtTheBeginning == collidingAtTheEnd: continue
				intervals.append((movingEdge, staticEdge, 0, 1))
		return intervals

	@classmethod
	def __getLineSegmentExpandedBb(cls, lineSeg: LineString, transformation: AffineTransform) -> Polygon:
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg)
		centerOfRotation = Geometry.intersection(lineSeg, finalConfig)
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
	def __getHalfIntervalObb(cls, movingEdge: LineString, region: AffineRegion, transformation: AffineTransform, intervalStart: float, intervalEnd: float) -> Tuple[Polygon, Polygon]:
		"""### Get Half Interval Obb
			given a moving edge -- along with the moving region that it belongs to --
			and returns the two subsequent OBB whose time interval is the first and second halves of the interval, respectively.

			Returns
			-------
			`Tuple[Polygon, Polygon]`
				Given the configuration information, it returns a tuple: `(Polygon, Polygon)`
				1. firstHalfBb, 2. secondHalfBb
		"""
		startConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalStart)
		edgeAtStart = Geometry.applyMatrixTransformToLineString(startConfigTransformation, movingEdge)
		endConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalEnd)
		intervalMid = (intervalStart + intervalEnd) / 2
		midConfigTransformation = Geometry.getParameterizedAffineTransformation(transformation, intervalMid)
		edgeAtMid = Geometry.applyMatrixTransformToLineString(midConfigTransformation, movingEdge)
		startSensor = Geometry.applyMatrixTransformToPolygon(startConfigTransformation, region.interior)
		midSensor = Geometry.applyMatrixTransformToPolygon(midConfigTransformation, region.interior)
		endSensor = Geometry.applyMatrixTransformToPolygon(endConfigTransformation, region.interior)
		firstHalfTransformation = Geometry.getAffineTransformation(startSensor, midSensor)
		secondHalfTransformation = Geometry.getAffineTransformation(midSensor, endSensor)
		firstHalfBb = cls.__getLineSegmentExpandedBb(edgeAtStart, firstHalfTransformation)
		secondHalfBb = cls.__getLineSegmentExpandedBb(edgeAtMid, secondHalfTransformation)
		return (firstHalfBb, secondHalfBb)

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
			`(madeCollisions, releasedCollisions)`
			New collisions that are made and released as the transformation is completed, respectively.
		"""
		madeCollisions: List[cls.CollisionInterval] = []
		releasedCollisions: List[cls.CollisionInterval] = []
		i = 0
		while i < len(collisionIntervals):
			(movingEdge, staticEdge, intervalStart, intervalEnd) = collisionIntervals.pop(i)
			# Epsilon for shard search
			deltaT = intervalEnd - intervalStart
			if deltaT <= cls.MIN_TIME_DELTA: continue # Minimum time interval assumption

			intervalMid = (intervalStart + intervalEnd) / 2
			collidingAtStart = cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalStart)
			collidingAtMid = cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalMid)
			collidingAtEnd = cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalEnd)
			# A change in the collision state indicates that we have found an actual collision.
			if collidingAtStart != collidingAtMid:
				if collidingAtStart and not collidingAtMid:
					releasedCollisions.append((movingEdge, staticEdge, intervalStart, intervalMid))
				else:
					madeCollisions.append((movingEdge, staticEdge, intervalStart, intervalMid))
			if collidingAtMid != collidingAtEnd:
				if collidingAtMid and not collidingAtEnd:
					releasedCollisions.append((movingEdge, staticEdge, intervalStart, intervalMid))
				else:
					madeCollisions.append((movingEdge, staticEdge, intervalStart, intervalMid))
			# No change in the collision state indicates that we have to further refine the time interval.
			if collidingAtStart == collidingAtMid and collidingAtMid == collidingAtEnd:
				(firstHalfBb, secondHalfBb) = cls.__getHalfIntervalObb(movingEdge, staticEdge, movingRegion, transformation, intervalStart, intervalEnd)
				if Geometry.intersects(firstHalfBb, staticEdge):
					collisionIntervals.insert(i, (movingEdge, staticEdge, intervalStart, intervalMid))
					i += 1
				if Geometry.intersects(secondHalfBb, staticEdge):
					collisionIntervals.insert(i, (movingEdge, staticEdge, intervalMid, intervalEnd))
					i += 1
		return (madeCollisions, releasedCollisions)

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
		intervals = cls.__initEdgeIntervals(movingRegion, transformation, startingCollisions, inverse)
		haveOverlap = intervals + estimatedCollision
		dontHaveOverlap: List[cls.CollisionInterval] = []
		i = 0
		while len(haveOverlap) > 0 and i < len(haveOverlap):
			(movingEdge, staticEdge, intervalStart, intervalEnd) = haveOverlap.pop(i)
			intervalMid = (intervalStart + intervalEnd) / 2
			collidingAtStart = cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalStart)
			collidingAtMid = cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalMid)
			collidingAtEnd = cls.__checkCollisionAtTime(movingEdge, staticEdge, transformation, movingRegion.centerOfRotation, intervalEnd)
			if collidingAtStart != collidingAtMid:
				haveOverlap.insert(i, (movingEdge, staticEdge, intervalStart, intervalMid))
				i += 1
			if collidingAtMid != collidingAtEnd:
				haveOverlap.insert(i, (movingEdge, staticEdge, intervalMid, intervalEnd))
				i += 1
			(haveOverlap, dontHaveOverlap) = cls.__splitIntervalsListForOverlap(haveOverlap)
			for interval in dontHaveOverlap:
				intermediateTransform = Geometry.getParameterizedAffineTransformation(transformation, interval[3])
				p = Geometry.applyMatrixTransformToPolygon(intermediateTransform, movingRegion.interior, movingRegion.centerOfRotation)
				eventCandidates.append((p, movingRegion.idNum, movingRegion.centerOfRotation, interval[3], "made" if inverse else "released", interval[1]))
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
		transformation = Geometry.getAffineTransformation(past.envelope, now.envelope)
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
	def __renderLineStrings(cls, lines: List[LineString], color: Color, renderWidth: float = 1.0) -> None:
		if cls.__rvizPublisher is None: return
		msg = MarkerArray()
		for line in lines:
			strId = repr(Geometry.lineStringId(line))
			marker = RViz.createLine(strId, Geometry.getGeometryCoords(line), color, renderWidth)
			# marker.lifetime = Duration(nanoseconds=(15 * (AffineRegion.NANO_CONSTANT / 10))).to_msg()
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
		cls.__renderLineStrings([polygon.exterior], KnownColors.BLUE, 2)
		for sensorId in pastSensors:
			if sensorId not in nowSensors:
				continue
			Logger().info("Estimating intermediate collisions between %s and the moving region %s." % (repr(polygon), sensorId))

			collisionData[sensorId] = cls.__getCollidingEdgesWithExtendedBb(pastSensors[sensorId], nowSensors[sensorId], polygon)
			pastCollidingEdges = cls.__getCollidingEdges(pastSensors[sensorId], polygon)
			nowCollidingEdges = cls.__getCollidingEdges(nowSensors[sensorId], polygon)
			transformation = Geometry.getAffineTransformation(pastSensors[sensorId].envelope, nowSensors[sensorId].envelope)

			# Following two liners are optimization lines and not necessary.
			collisionData[sensorId] = cls.__removeExistingCollisionsFromIntermediateCollisions(pastCollidingEdges, collisionData[sensorId])
			collisionData[sensorId] = cls.__removeExistingCollisionsFromIntermediateCollisions(nowCollidingEdges, collisionData[sensorId])
			if len(collisionData[sensorId]) == 0:
				Logger().info("Empty collision, skipping the refinement.")
				continue

			# Here we try to find the broadest set of collisions: any edge that collides with the OBB.
			# We also divide the collisions into two subgroups of ingoing and outgoing: from past to now and vice versa.
			madeCollisions: List[cls.CollisionInterval] = []
			releasedCollisions: List[cls.CollisionInterval] = []
			collisionIntervals = cls.__initEdgeIntervals(pastSensors[sensorId], transformation, pastCollidingEdges, inverse=False)
			Logger().debug("Initial Intervals: %s" % repr(collisionIntervals))
			while len(collisionIntervals) > 0:
				(madeCollisions, releasedCollisions) = cls.__estimateCollisionIntervals(pastSensors[sensorId], transformation, collisionIntervals)
			eventCandidates = cls.__refineIntervals(pastSensors[sensorId], transformation, pastCollidingEdges, releasedCollisions, [], inverse=False)
			eventCandidates = cls.__refineIntervals(nowSensors[sensorId], transformation, nowCollidingEdges, madeCollisions, eventCandidates, inverse=True)
			Logger().info("Total Events: %s" % repr(eventCandidates))
			for event in eventCandidates:
				(poly, _, _, _, _, lineString) = event
				cls.__renderLineStrings([poly.exterior, lineString], RViz.randomColor(), 2)
		return eventCandidates
