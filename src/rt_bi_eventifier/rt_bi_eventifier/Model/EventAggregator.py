from typing import TypeVar, cast

from networkx.algorithms.isomorphism import is_isomorphic

from rt_bi_core.Spatial import GraphPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion

_T_Poly = TypeVar("_T_Poly", bound=GraphPolygon)

class EventAggregator:
	"""
		This class contains functions related to Event Aggregator.
		For details see dissertation.
	"""

	@classmethod
	def __nodeNameMatcher(cls, n1: ConnectivityGraph.NodeData, n2: ConnectivityGraph.NodeData) -> bool:
		if not isinstance(n1.polygon, GraphPolygon): raise ValueError("No Region found.")
		if not isinstance(n2.polygon, GraphPolygon): raise ValueError("No Region found.")
		return n1.polygon.id == n2.polygon.id

	@classmethod
	def isIsomorphic(cls, g1: ConnectivityGraph, g2: ConnectivityGraph, useMatcher = False) -> bool:
		"""### Is Isomorphic
		Test whether two connectivity graphs are isomorphic.

		Parameters
		----------
		g1 : `ConnectivityGraph`
			Previous graph.
		g2 : `ConnectivityGraph`
			Next graph.
		useMatcher : `bool`, optional
			Whether to use the internal node mather function, by default `False`.

		Returns
		-------
		bool
			`True` if the graphs are isomorphic, `False` otherwise.
		"""
		if is_isomorphic(g1, g2, node_match= cls.__nodeNameMatcher if useMatcher else None):
			return True
		else:
			return False

	@classmethod
	def __interpolateTrack(cls, pastTracklet: Tracklet, nowTracklet: Tracklet, eventTimeNs: int, startTimeNs: int, endTimeNs: int) -> Tracklet:
		"""
			This function assumes members in previousTracks and currentTracks all have the same timestamp.
		"""
		if pastTracklet.id != nowTracklet.id: raise ValueError("Tracklets don't have the same id.. prevId = %d, currentId = %d" % (pastTracklet.id, nowTracklet.id))
		if eventTimeNs == startTimeNs: return pastTracklet
		if eventTimeNs == endTimeNs: return nowTracklet
		eventFraction = int((eventTimeNs - startTimeNs) / (endTimeNs - startTimeNs))
		x = ((eventFraction * (nowTracklet.x - pastTracklet.x)) + pastTracklet.x)
		y = ((eventFraction * (nowTracklet.y - pastTracklet.y)) + pastTracklet.y)
		psi = ((eventFraction * (nowTracklet.psi - pastTracklet.psi)) + pastTracklet.psi)
		Tracklet(nowTracklet.id, eventTimeNs, nowTracklet.hIndex, x, y, psi)
		return Tracklet(nowTracklet.id, eventTimeNs, nowTracklet.hIndex, x, y, psi)

	@classmethod
	def __interpolateRegion(cls, oldPolys: list[_T_Poly], newPolys: list[_T_Poly], eventTimeNs: int, excludeRegions: list[MovingPolygon.Id], tracklets: list[Tracklet] = []) -> list[_T_Poly]:
		regions: list[_T_Poly] = []
		for oldPoly in oldPolys:
			if oldPoly.id in excludeRegions: continue
			newPoly = next((p for p in newPolys if p.id.sansTime() == oldPoly.id.sansTime()), None)
			if newPoly is None: continue
			ctRegion = ContinuousTimeRegion[_T_Poly]([oldPoly, newPoly])
			oldPoly = ctRegion[eventTimeNs]
			if oldPoly.type == SensingPolygon.type:
				for track in tracklets: cast(SensingPolygon, oldPoly).tracklets[track.id] = track
			regions.append(oldPoly)
		return regions
