from typing import Sequence, TypeAlias

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial import GraphPolygon
from rt_bi_core.Spatial.ContinuousTimePolygon import ContinuousTimePolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon

CollisionInterval: TypeAlias = tuple[
	ContinuousTimePolygon[GraphPolygon], Shapely.LineString | None,
	ContinuousTimePolygon[GraphPolygon], Shapely.LineString | None,
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
	def __logInterval(cls, header: str, interval: CollisionInterval) -> None:
		(ctr1, edge1, ctr2, edge2, intervalStart, intervalEnd) = interval
		Ros.Log(header, [(ctr1, edge1), (ctr2, edge2), (intervalStart, intervalEnd)], indentStr="\t")
		return

	@classmethod
	def __renderLineStrings(cls, lines: list[Shapely.LineString], color: RGBA, renderWidth: float = 1.0) -> None:
		if cls.__rvizPublisher is None: return
		msg = RViz.Msgs.MarkerArray()
		for line in lines:
			markerId = RViz.Id(hIndex = -1, timeNanoSecs=-1, regionId="ctcd", polygonId=line.wkb_hex, subPartId="")
			marker = RViz.createLine(
				id=markerId,
				coordsList=GeometryLib.getGeometryCoords(line),
				outline=color,
				width=renderWidth,
			)
			Ros.AppendMessage(msg.markers, marker)
		cls.__rvizPublisher.publish(msg)
		return

	@classmethod
	def __obbTest(cls, ctRegion1: ContinuousTimePolygon[GraphPolygon], ctRegion2: ContinuousTimePolygon[GraphPolygon], upToNs: int) -> list[CollisionInterval]:
		# If there is no overlap in time then there are no collisions.
		if ctRegion1.latestNanoSecs < ctRegion2.earliestNanoSecs:
			Ros.Log(f"{repr(ctRegion1)} < {repr(ctRegion2)}")
			return []
		if ctRegion1.earliestNanoSecs > ctRegion2.latestNanoSecs:
			Ros.Log(f"{repr(ctRegion1)} > {repr(ctRegion2)}")
			return []

		Ros.Log(f"OBB Test: {ctRegion1.name} <==> {ctRegion2.name} -- up to {upToNs}")
		collisions: list[CollisionInterval] = []
		for e1 in ctRegion1.configs[ctRegion1.timeNanoSecsToIndex(upToNs)].edges:
			obb1 = ctRegion1.getEdgeBb(e1, upToNs)
			for e2 in ctRegion2.configs[ctRegion2.timeNanoSecsToIndex(upToNs)].edges:
				obb2 = ctRegion2.getEdgeBb(e2, upToNs)
				if GeometryLib.intersects(obb1, obb2): collisions.append(
					(
						ctRegion1, e1,
						ctRegion2, e2,
						max(ctRegion1.earliestNanoSecs, ctRegion2.earliestNanoSecs), upToNs
					)
				)
		return collisions

	@classmethod
	def __checkCollisionAtTime(cls, interval: CollisionInterval, timeNanoSecs: int) -> bool:
		(ctRegion1, edge1, ctRegion2, edge2, intervalStart, intervalEnd) = interval
		assert edge1 is not None and edge2 is not None, f"Does this ever happen? e1={edge1}, e2={edge2}"
		# Base case: delta T less than epsilon
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
	def __initTest(cls, ctRegion1: ContinuousTimePolygon[GraphPolygon], ctRegion2: ContinuousTimePolygon[GraphPolygon], upToNs: int) -> CollisionInterval | None:
		if upToNs not in ctRegion1: return
		if upToNs not in ctRegion2: return
		Ros.Log(f"INIT Test: {ctRegion1.name} <===> {ctRegion2.name} -- up to {upToNs}")
		if not GeometryLib.intersects(ctRegion1[upToNs].interior, ctRegion2[upToNs].interior):
			return
		interval = (
			ctRegion1, None,
			ctRegion2, None,
			upToNs, upToNs
		)
		return interval

	@classmethod
	def estimateCollisionIntervals(cls, ctrs: Sequence[ContinuousTimePolygon[GraphPolygon]], processUpToNs: int, rvizPublisher: Ros.Publisher | None) -> list[CollisionInterval]:
		cls.__rvizPublisher = rvizPublisher
		Ros.Log(" ------------------------------- CTCD - ESTIMATION - START --------------------------------")

		intervals: list[CollisionInterval] = []
		checked: set[tuple[SensingPolygon.Id, SensingPolygon.Id]] = set()
		for ctRegion1 in ctrs:
			if ctRegion1.isProjective:
				Ros.Log(f"Not testing {ctRegion1.name}: PROJECTIVE")
				continue
			if processUpToNs < ctRegion1.earliestNanoSecs:
				Ros.Log(f"Not testing {ctRegion1.name}: STARTS LATER.")
				continue
			for ctRegion2 in ctrs:
				if ctRegion1 == ctRegion2: continue
				if (ctRegion1.id, ctRegion2.id) in checked: continue

				if ctRegion1.isSlice:
					init = cls.__initTest(ctRegion1, ctRegion2, processUpToNs)
					if init is not None: intervals.append(init)
				elif not ctRegion2.isSlice and processUpToNs in ctRegion2:
					intervals += cls.__obbTest(ctRegion1, ctRegion2, processUpToNs)
				elif ctRegion2.isSlice:
					# If ctRegion2 is a slice, it will be tested when it is selected as ctRegion1 above.
					Ros.Log(f"Not testing {repr(ctRegion1)} vs {repr(ctRegion2)}: SLICE.")
				elif processUpToNs < ctRegion2.earliestNanoSecs:
					Ros.Log(f"Not testing {repr(ctRegion1)} vs {repr(ctRegion2)}: STARTS LATER.")
				else:
					raise AssertionError(f"Does this ever happen? ctr1 = {ctRegion1.name} vs ctr2 = {ctRegion2.name}")
				checked.add((ctRegion1.id, ctRegion2.id))
				checked.add((ctRegion2.id, ctRegion1.id))
		Ros.Log(" ------------------------------- CTCD - ESTIMATION - END ----------------------------------")
		return intervals

	@classmethod
	def refineCollisionIntervals(cls, intervals: list[CollisionInterval]) -> list[CollisionInterval]:
		if len(intervals) < 2: return intervals
		Ros.Log(" ------------------------------- CTCD - REFINEMENT - START --------------------------------")
		Ros.Log(f"{len(intervals)} intervals.")
		withOverlap = intervals.copy()
		withoutOverlap: list[CollisionInterval] = []
		i = 0
		while len(withOverlap) > 0 and i < len(withOverlap):
			cls.__logInterval("Interval to Refine", withOverlap[i])
			(ctRegion1, edge1, ctRegion2, edge2, intervalStart, intervalEnd) = withOverlap[i]
			if edge1 is not None and edge2 is not None:
				interval = withOverlap.pop(i)
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
		for i in range(len(withoutOverlap)): cls.__logInterval(f"Non-overlapping Interval {i}", withoutOverlap[i])
		Ros.Log(" ------------------------------- CTCD - REFINEMENT - END ----------------------------------")
		return withoutOverlap
