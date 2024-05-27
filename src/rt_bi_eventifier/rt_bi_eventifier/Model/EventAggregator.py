# from typing import TypeVar, cast


# from rt_bi_core.Spatial import GraphPolygon
# from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
# from rt_bi_core.Spatial.ContinuousTimePolygon import ContinuousTimePolygon
# from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
# from rt_bi_core.Spatial.Tracklet import Tracklet
# from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph

# _T_Poly = TypeVar("_T_Poly", bound=GraphPolygon)

# class EventAggregator:
# 	"""
# 		This class contains functions related to Event Aggregator.
# 		For details see dissertation.
# 	"""

# 	@classmethod
# 	def __nodeNameMatcher(cls, n1: ConnectivityGraph.NodeData, n2: ConnectivityGraph.NodeData) -> bool:
# 		if not isinstance(n1.polygon, GraphPolygon): raise ValueError("No Region found.")
# 		if not isinstance(n2.polygon, GraphPolygon): raise ValueError("No Region found.")
# 		return n1.polygon.id == n2.polygon.id

# 	@classmethod
# 	def __interpolateRegion(cls, oldPolys: list[_T_Poly], newPolys: list[_T_Poly], eventTimeNs: int, excludeRegions: list[AffinePolygon.Id], tracklets: list[Tracklet] = []) -> list[_T_Poly]:
# 		regions: list[_T_Poly] = []
# 		for oldPoly in oldPolys:
# 			if oldPoly.id in excludeRegions: continue
# 			newPoly = next((p for p in newPolys if p.id.sansTime() == oldPoly.id.sansTime()), None)
# 			if newPoly is None: continue
# 			ctRegion = ContinuousTimePolygon[_T_Poly]([oldPoly, newPoly])
# 			oldPoly = ctRegion[eventTimeNs]
# 			if oldPoly.type == SensingPolygon.type:
# 				for track in tracklets: cast(SensingPolygon, oldPoly).tracklets[track.id] = track
# 			regions.append(oldPoly)
# 		return regions
