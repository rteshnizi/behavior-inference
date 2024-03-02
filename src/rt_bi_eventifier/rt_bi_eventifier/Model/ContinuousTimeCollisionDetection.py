from typing import TypeAlias

from rclpy.clock import Duration

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.Ros import AppendMessage, Log, Publisher
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion

RegionCollisions = dict[str, list[Shapely.LineString]]
"""
Represents all the collisions a Region and a Polygon.
### Type
```
{"edgeId": {L1, L2, ...}}
dict[str, list[Shapely.LineString]]
```
wherein, `edgeId` is the id of the region edge and `Li` is the edge of the other.
"""

RegularRegionCollisions = dict[str, RegionCollisions]
"""
Represents all the collisions between a regular region and a polygon.
### Type
```
{"subRegionName": {"edgeId": {L1, L2, ...}}}
dict[str, dict[str, list[Shapely.LineString]]]
```
"""

_InputPolyTypes: TypeAlias = MovingPolygon | StaticPolygon | SensingPolygon

CollisionInterval = tuple[ContinuousTimeRegion[_InputPolyTypes], str, ContinuousTimeRegion[_InputPolyTypes], str, int, int]

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

	MIN_TIME_DELTA_NS = 25
	"""
	### A Core Assumption:
	We expect the updates to be at least as fast as `MIN_TIME_DELTA` ns.
	"""

	__rvizPublisher: Publisher | None = None

	@classmethod
	def __renderLineStrings(cls, lines: list[Shapely.LineString], color: RGBA, renderWidth: float = 1.0) -> None:
		if cls.__rvizPublisher is None: return
		msg = RViz.Msgs.MarkerArray()
		for line in lines:
			strId = repr(GeometryLib.lineStringId(line))
			marker = RViz.createLine(MovingPolygon.Id("ctcd", strId, "", ""), GeometryLib.getGeometryCoords(line), color, renderWidth)
			marker.lifetime = Duration(nanoseconds=int(15 * (MovingPolygon.NANO_CONVERSION_CONSTANT / 10))).to_msg()
			AppendMessage(msg.markers, marker)
		cls.__rvizPublisher.publish(msg)
		return

	@classmethod
	def __obbTest(cls, ctRegion1: ContinuousTimeRegion, ctRegion2: ContinuousTimeRegion) -> list[CollisionInterval]:
		# If there is no overlap in time then there are no collisions.
		if ctRegion1.latestNanoSecs < ctRegion2.earliestNanoSecs:
			Log(f"{repr(ctRegion1)} < {repr(ctRegion2)}")
			return []
		if ctRegion1.earliestNanoSecs > ctRegion2.latestNanoSecs:
			Log(f"{repr(ctRegion1)} > {repr(ctRegion2)}")
			return []

		collisions: list[CollisionInterval] = []
		for eName1 in ctRegion1.configs[0].edges:
			obb1 = ctRegion1.getEdgeBb(eName1)
			for eName2 in ctRegion2.configs[0].edges:
				obb2 = ctRegion2.getEdgeBb(eName2)
				if GeometryLib.intersects(obb1, obb2): collisions.append(
					(
						ctRegion1, eName1,
						ctRegion2, eName2,
						max(ctRegion1.earliestNanoSecs, ctRegion2.earliestNanoSecs), min(ctRegion1.latestNanoSecs, ctRegion2.latestNanoSecs)
					)
				)
		return collisions

	@classmethod
	def __checkCollisionAtTime(cls, interval: CollisionInterval, timeNanoSecs: int) -> bool:
		(ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalStart, intervalEnd) = interval
		# Base case FIXME: and delta T less than epsilon
		if intervalEnd - intervalStart < cls.MIN_TIME_DELTA_NS: return False

		e1AtT = ctRegion1.getEdgeAt(edgeStrId1, timeNanoSecs)
		e2AtT = ctRegion2.getEdgeAt(edgeStrId2, timeNanoSecs)
		return GeometryLib.intersects(e1AtT, e2AtT)

	@classmethod
	def __checkIntervalsForOverlap(cls, interval1: tuple[int, int], interval2: tuple[int, int]) -> bool:
		# If end of one interval happens earlier than the other
		if interval1[1] <= interval2[0]: return False
		if interval2[1] <= interval1[0]: return False
		return True

	@classmethod
	def __splitIntervalsListForOverlap(cls, intervals: list[CollisionInterval]) -> tuple[list[CollisionInterval], list[CollisionInterval]]:
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
	def __initTest(cls, ctRegion1: ContinuousTimeRegion, ctRegion2: ContinuousTimeRegion) -> CollisionInterval | None:
		if ctRegion1.length == 0: return
		t = ctRegion1.earliestNanoSecs
		if t not in ctRegion2: return
		r1 = ctRegion1[t]
		if not GeometryLib.intersects(r1.interior, ctRegion2[t].interior): return
		Log(f"Adding init interval: {ctRegion1.name} <===> {ctRegion2.name} @ {t}")
		interval = (
			ctRegion1, "",
			ctRegion2, "",
			t, t
		)
		return interval

	@classmethod
	def estimateCollisionIntervals(cls, sensorPolys: list[SensingPolygon], mapPolys: list[MovingPolygon | StaticPolygon], rvizPublisher: Publisher | None) -> list[CollisionInterval]:
		cls.__rvizPublisher = rvizPublisher
		Log(f"CtCd: {repr(sensorPolys)} and {repr(mapPolys)}")
		allCtRegions = ContinuousTimeRegion[SensingPolygon].fromMergedList(sensorPolys) + ContinuousTimeRegion[MovingPolygon | StaticPolygon].fromMergedList(mapPolys)

		intervals = []
		checked: set[tuple[str, str]] = set()
		for ctRegion1 in allCtRegions:
			if ctRegion1.type != StaticPolygon.type: continue
			for ctRegion2 in allCtRegions:
				if ctRegion1 == ctRegion2: continue
				if (ctRegion1.name, ctRegion2.name) in checked: continue

				if ctRegion1.isSlice:
					init = cls.__initTest(ctRegion1, ctRegion2) # type: ignore
					if init is not None: intervals.append(init)
				elif not ctRegion2.isSlice:
					intervals += cls.__obbTest(ctRegion1, ctRegion2) # type: ignore
				checked.add((ctRegion1.name, ctRegion2.name))
				checked.add((ctRegion2.name, ctRegion1.name))
		return intervals

	@classmethod
	def refineCollisionIntervals(cls, intervals: list[CollisionInterval]) -> list[CollisionInterval]:
		withOverlap = intervals.copy()
		withoutOverlap: list[CollisionInterval] = []
		i = 0
		while len(withOverlap) > 0 and i < len(withOverlap):
			interval = withOverlap.pop(i)
			(ctRegion1, edgeStrId1, ctRegion2, edgeStrId2, intervalStart, intervalEnd) = interval
			if edgeStrId1 != "" or edgeStrId2 != "":
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

		return withoutOverlap
