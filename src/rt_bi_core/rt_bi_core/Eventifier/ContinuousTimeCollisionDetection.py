from typing import Dict, List, Tuple, TypeVar, Union

from rclpy.clock import Duration
from visualization_msgs.msg import MarkerArray

from rt_bi_core.Eventifier.ContinuousTimeRegion import ContinuousTimeRegion
from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.Ros import AppendMessage, Logger, Publisher
from rt_bi_utils.RViz import Color, RViz

RegionTypeX = TypeVar("RegionTypeX", bound=AffineRegion)
RegionTypeY = TypeVar("RegionTypeY", bound=AffineRegion)

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

CollisionInterval = Tuple[ContinuousTimeRegion[RegionTypeX], str, ContinuousTimeRegion[RegionTypeY], str, int, int]
"""## Collision Interval

A collision interval is a represents an interval of time in which two edges collide.
`[ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, T1, T2]`
`ctRegion1` and `ctRegion2` move and a collision will happen between edges with Id `edgeStrId1` and `edgeStrId2`, respectively.
The two `int`s are the bounds of the interval of time when this event happens: `[T1, T2)`.
`T1` and `T2` are absolute values of time in NanoSeconds as an integer.
"""

class ContinuousTimeCollisionDetection:
	"""
		This class contains functions related to Continuous-time Collision Detection.
		© Reza Teshnizi 2018-2023
	"""

	MIN_TIME_DELTA = 1
	"""
	### A Core Assumption:
	We expect the updates to be at least as fast as `MIN_TIME_DELTA` ns.
	"""

	__rvizPublisher: Union[Publisher, None] = None

	@classmethod
	def __renderLineStrings(cls, lines: List[LineString], color: Color, renderWidth: float = 1.0) -> None:
		if cls.__rvizPublisher is None: return
		msg = MarkerArray()
		for line in lines:
			strId = repr(Geometry.lineStringId(line))
			marker = RViz.createLine(strId, Geometry.getGeometryCoords(line), color, renderWidth)
			marker.lifetime = Duration(nanoseconds=int(15 * (AffineRegion.NANO_CONSTANT / 10))).to_msg()
			AppendMessage(msg.markers, marker)
		cls.__rvizPublisher.publish(msg)
		return

	@classmethod
	def __expandVertObbWithAngularVelocity(cls, coords: Geometry.Coords, angle: float, centerOfRotation: Pose, expandAway = True) -> Geometry.Coords:
		displacement = (coords[0] - centerOfRotation.x, coords[1] - centerOfRotation.y)
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	@classmethod
	def __getLineSegmentExpandedBb(cls, transformation: AffineTransform, lineSeg: LineString, centerOfRotation: Pose) -> Union[Polygon, LineString]:
		""" Gets a tight bounding box for a line segment that is moving with a constant angular velocity. """
		angle: float = abs(transformation.rotation)
		originalCoords = Geometry.getGeometryCoords(lineSeg)
		if len(originalCoords) > 2: raise ValueError("A line segment must have two vertices. Input: %s" % repr(lineSeg))
		if Geometry.isIdentityTransform(transformation):
			return lineSeg
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg)
		finalCoords = Geometry.getGeometryCoords(finalConfig)
		verts: Geometry.CoordsList = []
		for j in range(16):
			v1 = cls.__expandVertObbWithAngularVelocity(originalCoords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = cls.__expandVertObbWithAngularVelocity(finalCoords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = cls.__expandVertObbWithAngularVelocity(finalCoords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = cls.__expandVertObbWithAngularVelocity(originalCoords[1], angle, centerOfRotation, j & 8 != 0)
			verts += [v1, v2, v3, v4]
		expandedObb = Geometry.convexHull(verts)
		if isinstance(expandedObb, Polygon): return expandedObb
		raise ValueError("Expanded OBB is not a polygon: %s" % repr(expandedObb))

	@classmethod
	def __obbTest(cls, ctRegion1: ContinuousTimeRegion[RegionTypeX], ctRegion2: ContinuousTimeRegion[RegionTypeY]) -> List[CollisionInterval[RegionTypeX, RegionTypeY]]:
		# If there is no overlap in time then there are no collisions.
		if ctRegion1.latestNanoSecs < ctRegion2.earliestNanoSecs:
			Logger().debug("ctRegion1.latest < ctRegion2.earliest --> %d < %d" % (ctRegion1.latestNanoSecs, ctRegion2.earliestNanoSecs))
			return []
		if ctRegion1.earliestNanoSecs > ctRegion2.latestNanoSecs:
			Logger().debug("ctRegion1.earliest > ctRegion2.latest --> %d > %d" % (ctRegion1.earliestNanoSecs, ctRegion2.latestNanoSecs))
			return []

		collisions: List[CollisionInterval] = []
		for eName1 in ctRegion1.configs[0].edges:
			edge1 = ctRegion1.configs[0].edges[eName1]
			for eName2 in ctRegion2.configs[0].edges:
				edge2 = ctRegion2.configs[0].edges[eName2]
				obb1 = cls.__getLineSegmentExpandedBb(ctRegion1.transformations[0], edge1, ctRegion1.configs[0].centerOfRotation)
				obb2 = cls.__getLineSegmentExpandedBb(ctRegion2.transformations[0], edge2, ctRegion2.configs[0].centerOfRotation)
				edge1Id = ctRegion1.configs[0].edgeId(edge1)
				if edge1Id is None: raise KeyError("Edge doesn't exist in ctRegion1. E: %s, R: %s" % (repr(edge1), repr(ctRegion1)))
				edge2Id = ctRegion2.configs[0].edgeId(edge2)
				if edge2Id is None: raise KeyError("Edge doesn't exist in ctRegion2. E: %s, R: %s" % (repr(edge2), repr(ctRegion2)))
				collision = (
					ctRegion1, edge1Id,
					ctRegion2, edge2Id,
					ctRegion1.earliestNanoSecs, ctRegion1.earliestNanoSecs
				)
				if Geometry.intersects(obb1, obb2): collisions.append(collision)
		return collisions

	@classmethod
	def __checkCollisionAtTime(cls, interval: CollisionInterval[RegionTypeX, RegionTypeY], timeNanoSecs: int) -> bool:
		(ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalStart, intervalEnd) = interval
		e1 = ctRegion1.configs[0].edges[edgeStrId1]
		e2 = ctRegion2.configs[0].edges[edgeStrId2]
		# Base case
		if intervalStart == intervalEnd: return Geometry.intersects(e1, e2)

		intervalFraction = ((timeNanoSecs - intervalStart) / (intervalEnd - intervalStart))
		transformation1AtT = Geometry.getParameterizedAffineTransformation(ctRegion1.transformations[0], intervalFraction)
		e1AtT = Geometry.applyMatrixTransformToLineString(transformation1AtT, e1)
		transformation2AtT = Geometry.getParameterizedAffineTransformation(ctRegion2.transformations[0], intervalFraction)
		e2AtT = Geometry.applyMatrixTransformToLineString(transformation2AtT, e2)
		return Geometry.intersects(e1AtT, e2AtT)

	@classmethod
	def __checkIntervalsForOverlap(cls, interval1: Tuple[int, int], interval2: Tuple[int, int]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	@classmethod
	def __splitIntervalsListForOverlap(cls, intervals: List[CollisionInterval[RegionTypeX, RegionTypeY]]) -> Tuple[List[CollisionInterval[RegionTypeX, RegionTypeY]], List[CollisionInterval[RegionTypeX, RegionTypeY]]]:
		haveOverlap = []
		dontHaveOverlap = []
		while len(intervals) > 0:
			interval1 = intervals.pop()
			foundOverlap = False
			for interval2 in intervals:
				if interval1 == interval2: continue
				(_, _, _, _, interval1Start, interval1End) = interval1
				(_, _, _, _, interval2Start, interval2End) = interval2
				if cls.__checkIntervalsForOverlap((interval1Start, interval1End), (interval2Start, interval2End)):
					foundOverlap = True
					haveOverlap.append(interval1)
					break
			if not foundOverlap:
				dontHaveOverlap.append(interval1)
		return (haveOverlap, dontHaveOverlap)

	@classmethod
	def estimateCollisionsIntervals(cls, r1Configs: Tuple[RegionTypeX, RegionTypeX], r2Configs: Tuple[RegionTypeY, RegionTypeY], rvizPublisher: Union[Publisher, None]) -> List[CollisionInterval[RegionTypeX, RegionTypeY]]:
		"""## Detection Collisions

		Parameters
		----------
		r1Configs : `RegularRegionPair`
			Two snapshots of a continuous time region
		r2Configs : `RegularRegionPair`
			Two snapshots of a continuous time region
		rvizPublisher : Union[Publisher, None]
			RViz publisher for debugging

		Returns
		-------
		List[Collision]
			List of collisions.
		"""
		cls.__rvizPublisher = rvizPublisher
		(r1Past, r1Now) = r1Configs
		(r2Past, r2Now) = r2Configs
		ctRegion1 = ContinuousTimeRegion[RegionTypeX](r1Past, r1Now)
		ctRegion2 = ContinuousTimeRegion[RegionTypeY](r2Past, r2Now)
		Logger().debug("[1] CtCd: %s <---> %s" % (repr(ctRegion1), repr(ctRegion2)))
		intervals = cls.__obbTest(ctRegion1, ctRegion2)
		Logger().debug("[2] CtCd: intervals %d" % (len(intervals)))
		return intervals

	@classmethod
	def refineCollisionIntervals(cls, intervals: List[CollisionInterval]) -> List[CollisionInterval]:
		withOverlap = intervals.copy()
		withoutOverlap: List[CollisionInterval] = []
		i = 0
		while len(withOverlap) > 0 and i < len(withOverlap):
			interval = withOverlap.pop(i)
			(ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalStart, intervalEnd) = interval
			intervalMid = int((intervalStart + intervalEnd) / 2)
			collidingAtStart = cls.__checkCollisionAtTime(interval, intervalStart)
			collidingAtMid = cls.__checkCollisionAtTime(interval, intervalMid)
			collidingAtEnd = cls.__checkCollisionAtTime(interval, intervalEnd)
			if collidingAtStart != collidingAtMid:
				withOverlap.insert(i, (ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalStart, intervalMid))
				i += 1
			if collidingAtMid != collidingAtEnd:
				withOverlap.insert(i, (ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalMid, intervalEnd))
				i += 1
			(withOverlap, withoutOverlap) = cls.__splitIntervalsListForOverlap(withOverlap)

		obbListToRender = []
		for interval in withoutOverlap:
			(ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalStart, intervalEnd) = interval
			obb1 = cls.__getLineSegmentExpandedBb(ctRegion1.transformations[0], ctRegion1.configs[0].edges[edgeStrId1], ctRegion1.configs[0].centerOfRotation)
			obb2 = cls.__getLineSegmentExpandedBb(ctRegion2.transformations[0], ctRegion2.configs[0].edges[edgeStrId2], ctRegion2.configs[0].centerOfRotation)
			obbListToRender.append(obb1.exterior if isinstance(obb1, Polygon) else obb1)
			obbListToRender.append(obb2.exterior if isinstance(obb2, Polygon) else obb2)
		cls.__renderLineStrings(obbListToRender, RViz.randomColor(), 2)
		return withoutOverlap
