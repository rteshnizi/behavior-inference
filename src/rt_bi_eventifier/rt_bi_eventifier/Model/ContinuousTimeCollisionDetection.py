from typing import TypeAlias

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import GraphInputPolygon, MapPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion

CollisionInterval: TypeAlias = tuple[
	ContinuousTimeRegion[GraphInputPolygon], Shapely.LineString | None,
	ContinuousTimeRegion[GraphInputPolygon], Shapely.LineString | None,
	int, int
]
"""## Collision Interval

A collision interval represents an interval of time in which two edges collide.
`[ctRegion1, edge1, ctRegion2, edge2, T1, T2]`
`ctRegion1` and `ctRegion2` move and a collision will happen between edges `edge1` and `edge2`, from the respective region.
The two `int`s are the bounds of the interval of time when this event happens: `[T1, T2)`.
`T1` and `T2` are absolute values of time in NanoSeconds as an integer.
"""

class ContinuousTimeCollisionDetection:
	"""
		This class contains functions related to Continuous-time Collision Detection.
	"""

	MIN_TIME_DELTA_NS = 25
	"""
	### A Core Assumption:
	We expect the updates to be at least as fast as `MIN_TIME_DELTA` ns.
	"""

	__rvizPublisher: Ros.Publisher | None = None

	@classmethod
	def __renderLineStrings(cls, lines: list[Shapely.LineString], color: RGBA, renderWidth: float = 1.0) -> None:
		if cls.__rvizPublisher is None: return
		msg = RViz.Msgs.MarkerArray()
		for line in lines:
			strId = repr(GeometryLib.lineStringId(line))
			markerId = MovingPolygon.Id(timeNanoSecs=-1, regionId="ctcd", polygonId=strId)
			marker = RViz.createLine(
				id=markerId,
				coords=GeometryLib.getGeometryCoords(line),
				outline=color,
				width=renderWidth,
			)
			Ros.AppendMessage(msg.markers, marker)
		cls.__rvizPublisher.publish(msg)
		return

	@classmethod
	def __obbTest(cls, ctRegion1: ContinuousTimeRegion, ctRegion2: ContinuousTimeRegion) -> list[CollisionInterval]:
		# If there is no overlap in time then there are no collisions.
		if ctRegion1.latestNanoSecs < ctRegion2.earliestNanoSecs:
			Ros.Log(f"{repr(ctRegion1)} < {repr(ctRegion2)}")
			return []
		if ctRegion1.earliestNanoSecs > ctRegion2.latestNanoSecs:
			Ros.Log(f"{repr(ctRegion1)} > {repr(ctRegion2)}")
			return []

		collisions: list[CollisionInterval] = []
		for e1 in ctRegion1.configs[0].edges:
			obb1 = ctRegion1.getEdgeBb(e1)
			for e2 in ctRegion2.configs[0].edges:
				obb2 = ctRegion2.getEdgeBb(e2)
				if GeometryLib.intersects(obb1, obb2): collisions.append(
					(
						ctRegion1, e1,
						ctRegion2, e2,
						max(ctRegion1.earliestNanoSecs, ctRegion2.earliestNanoSecs), min(ctRegion1.latestNanoSecs, ctRegion2.latestNanoSecs)
					)
				)
		return collisions

	@classmethod
	def __checkCollisionAtTime(cls, interval: CollisionInterval, timeNanoSecs: int) -> bool:
		(ctRegion1, edge1, ctRegion2, edge2, intervalStart, intervalEnd) = interval
		# Base case FIXME: and delta T less than epsilon
		if intervalEnd - intervalStart < cls.MIN_TIME_DELTA_NS: return False

		e1AtT = ctRegion1.getEdgeAt(edge1, timeNanoSecs)
		e2AtT = ctRegion2.getEdgeAt(edge2, timeNanoSecs)
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
		Ros.Log(f"Adding init interval: {ctRegion1.name} <===> {ctRegion2.name} @ {t}")
		interval = (
			ctRegion1, None,
			ctRegion2, None,
			t, t
		)
		return interval

	@classmethod
	def estimateCollisionIntervals(cls, sensorPolys: list[SensingPolygon], mapPolys: list[MapPolygon], rvizPublisher: Ros.Publisher | None) -> list[CollisionInterval]:
		cls.__rvizPublisher = rvizPublisher
		Ros.Log("<===================================CTCD - START===================================>")
		Ros.Log(f"Sensors: {repr(sensorPolys)}")
		Ros.Log(f"Map {repr(mapPolys)}")
		ctSensors = ContinuousTimeRegion[SensingPolygon].fromMergedList(sensorPolys)
		ctMap = ContinuousTimeRegion[MapPolygon].fromMergedList(mapPolys)
		allCtRegions = ctSensors + ctMap
		Ros.Log(f"CT Sensors: {len(ctSensors)}")
		Ros.Log(f"CT Map: {len(ctMap)}")

		intervals = []
		checked: set[tuple[SensingPolygon.Id, SensingPolygon.Id]] = set()
		for ctRegion1 in allCtRegions:
			if ctRegion1.type != StaticPolygon.type: continue
			for ctRegion2 in allCtRegions:
				if ctRegion1 == ctRegion2: continue
				if (ctRegion1.id, ctRegion2.id) in checked: continue

				if ctRegion1.isSlice:
					init = cls.__initTest(ctRegion1, ctRegion2)
					if init is not None: intervals.append(init)
				elif not ctRegion2.isSlice:
					intervals += cls.__obbTest(ctRegion1, ctRegion2)
				checked.add((ctRegion1.id, ctRegion2.id))
				checked.add((ctRegion2.id, ctRegion1.id))
		Ros.Log("<===================================CTCD - END=====================================>")
		return intervals

	@classmethod
	def refineCollisionIntervals(cls, intervals: list[CollisionInterval]) -> list[CollisionInterval]:
		withOverlap = intervals.copy()
		withoutOverlap: list[CollisionInterval] = []
		i = 0
		while len(withOverlap) > 0 and i < len(withOverlap):
			interval = withOverlap.pop(i)
			(ctRegion1, edge1, ctRegion2, edge2, intervalStart, intervalEnd) = interval
			if edge1 is None or edge2 is None: continue
			intervalMid = int((intervalStart + intervalEnd) / 2)
			collidingAtStart = cls.__checkCollisionAtTime(interval, intervalStart)
			collidingAtMid = cls.__checkCollisionAtTime(interval, intervalMid)
			collidingAtEnd = cls.__checkCollisionAtTime(interval, intervalEnd)
			if collidingAtStart != collidingAtMid:
				withOverlap.insert(i, (ctRegion1, edge1, ctRegion2, edge2, intervalStart, intervalMid))
				i += 1
			if collidingAtMid != collidingAtEnd:
				withOverlap.insert(i, (ctRegion1, edge1, ctRegion2, edge2, intervalMid, intervalEnd))
				i += 1
			(withOverlap, withoutOverlap) = cls.__splitIntervalsListForOverlap(withOverlap)

		return withoutOverlap
