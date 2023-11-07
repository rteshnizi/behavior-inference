from typing import Dict, List, Literal, Tuple, TypeVar, Union

from rclpy.clock import Duration
from visualization_msgs.msg import MarkerArray

from rt_bi_core.Eventifier.ContinuousTimeRegion import ContinuousTimeRegion
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.TimeRegion import TimeRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.Ros import AppendMessage, Logger, Publisher
from rt_bi_utils.RViz import Color, KnownColors, RViz


class ContinuousTimeCollisionDetection:
	"""
		This class contains functions related to Continuous-time Collision Detection.
		© Reza Teshnizi 2018-2023
	"""

	Collision = Tuple[Polygon, TimeRegion, Literal["made", "released"], LineString]
	"""### Collision
	```
	(Polygon, timeOfEvent, "made" | "released", collidingEdge)
	```
	Wherein:
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

	CollisionInterval = Tuple[LineString, LineString, int, int]
	"""## Collision Interval

	A collision interval is a represents an interval of time in which two edges collide.
	A tuple `(L1, L2, T1, T2)`, the two floats are the bounds of the interval of time when this event happens: `[T1, T2)`.
	`T1` and `T2` are absolute values of time in NanoSeconds as an integer.
	"""

	RegionType = TypeVar("RegionType", bound=DynamicRegion)

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
			marker.lifetime = Duration(nanoseconds=int(15 * (DynamicRegion.NANO_CONSTANT / 10))).to_msg()
			AppendMessage(msg.markers, marker)
		cls.__rvizPublisher.publish(msg)
		return

	@classmethod
	def __expandVertObbWithAngularVelocity(cls, coords: Geometry.Coords, angle: float, centerOfRotation: Geometry.Coords, expandAway = True) -> Geometry.Coords:
		displacement = (coords[0] - centerOfRotation[0], coords[1] - centerOfRotation[1])
		vertExpansion = (angle * displacement[0], angle * displacement[1])
		expanded = (coords[0] + vertExpansion[0], coords[1] + vertExpansion[1]) if expandAway else (coords[0] - vertExpansion[0], coords[1] - vertExpansion[1])
		return expanded

	@classmethod
	def __getLineSegmentExpandedBb(cls, transformation: AffineTransform, lineSeg: LineString) -> Union[Polygon, LineString]:
		""" Gets a tight bounding box for a line segment that is moving with a constant angular velocity. """
		angle: float = abs(transformation.rotation)
		originalCoords = Geometry.getGeometryCoords(lineSeg)
		if len(originalCoords) > 2: raise ValueError("A line segment must have two vertices. Input: %s" % repr(lineSeg))
		if Geometry.isIdentityTransform(transformation):
			return lineSeg
		centerOfRotation = Geometry.getCenterOfRotation(transformation)
		finalConfig = Geometry.applyMatrixTransformToLineString(transformation, lineSeg)
		finalCoords = Geometry.getGeometryCoords(finalConfig)
		polygons = []
		for j in range(16):
			v1 = cls.__expandVertObbWithAngularVelocity(originalCoords[0], angle, centerOfRotation, j & 1 != 0)
			v2 = cls.__expandVertObbWithAngularVelocity(finalCoords[0], angle, centerOfRotation, j & 2 != 0)
			v3 = cls.__expandVertObbWithAngularVelocity(finalCoords[1], angle, centerOfRotation, j & 4 != 0)
			v4 = cls.__expandVertObbWithAngularVelocity(originalCoords[1], angle, centerOfRotation, j & 8 != 0)
			p = Polygon([v1, v2, v3, v4])
			polygons.append(p)
		expandedObb = Geometry.union(polygons)
		expandedObb: Polygon = expandedObb.convex_hull
		if isinstance(expandedObb, Polygon): return expandedObb
		raise ValueError("Expanded OBB is not a polygon: %s" % repr(expandedObb))

	@classmethod
	def __obbTest(cls, ctRegion1: ContinuousTimeRegion[RegionType], ctRegion2: ContinuousTimeRegion[RegionType]) -> List[CollisionInterval]:
		# If there is no overlap in time then there are no collisions.
		if ctRegion1.latestNanoSecs < ctRegion2.earliestNanoSecs:
			Logger().debug("ctRegion1.latest < ctRegion2.earliest --> %d < %d" % (ctRegion1.latestNanoSecs, ctRegion2.earliestNanoSecs))
			return []
		if ctRegion1.earliestNanoSecs > ctRegion2.latestNanoSecs:
			Logger().debug("ctRegion1.earliest > ctRegion2.latest --> %d > %d" % (ctRegion1.earliestNanoSecs, ctRegion2.latestNanoSecs))
			return []

		collisions: List[ContinuousTimeCollisionDetection.CollisionInterval] = []
		obbListToRender = []
		for eName1 in ctRegion1.configs[0].edges:
			edge1 = ctRegion1.configs[0].edges[eName1]
			for eName2 in ctRegion2.configs[0].edges:
				edge2 = ctRegion2.configs[0].edges[eName2]
				obb1 = cls.__getLineSegmentExpandedBb(ctRegion1.transformations[0], edge1)
				obb2 = cls.__getLineSegmentExpandedBb(ctRegion2.transformations[0], edge2)
				obbListToRender.append(obb1.exterior if isinstance(obb1, Polygon) else obb1)
				obbListToRender.append(obb2.exterior if isinstance(obb2, Polygon) else obb2)
				collision = (edge1, edge2, ctRegion1.earliestNanoSecs, ctRegion1.earliestNanoSecs)
				if Geometry.intersects(obb1, obb2): collisions.append(collision)
		cls.__renderLineStrings(obbListToRender, RViz.randomColor(), 2)
		return collisions

	@classmethod
	def estimateCollisionsIntervals(cls, r1Configs: Tuple[RegionType, RegionType], r2Configs: Tuple[RegionType, RegionType], rvizPublisher: Union[Publisher, None]) -> List[CollisionInterval]:
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
		ctRegion1 = ContinuousTimeRegion[cls.RegionType](r1Past, r1Now)
		ctRegion2 = ContinuousTimeRegion[cls.RegionType](r2Past, r2Now)
		intervals = cls.__obbTest(ctRegion1, ctRegion2)
		return intervals

	@classmethod
	def refineCollisionIntervals(cls, interval: List[CollisionInterval]) -> List[Collision]:
		return []
