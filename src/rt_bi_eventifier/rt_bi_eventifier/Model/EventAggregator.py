from typing import List, TypeAlias, TypeVar, cast

from networkx.algorithms.isomorphism import is_isomorphic

from rt_bi_commons.Utils import Ros
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.SpatialRegion import SpatialRegion
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import CollisionInterval
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion

_InputPolyTypes: TypeAlias = MovingPolygon | StaticPolygon | SensingPolygon
_T_Poly = TypeVar("_T_Poly", bound=_InputPolyTypes)

class EventAggregator:
	"""
		This class contains functions related to Event Aggregator.
		For details see dissertation.
		© Reza Teshnizi 2018-2023
	"""

	@classmethod
	def __nodeNameMatcher(cls, n1: ConnectivityGraph.NodeContent, n2: ConnectivityGraph.NodeContent) -> bool:
		n1Content = n1["polygon"]
		n2Content = n2["polygon"]
		if not isinstance(n1Content, _InputPolyTypes): raise ValueError("No Region found.")
		if not isinstance(n2Content, _InputPolyTypes): raise ValueError("No Region found.")
		return n1Content.id == n2Content.id

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
		Tracklet(nowTracklet.id, eventTimeNs, x, y, psi)
		return Tracklet(nowTracklet.id, eventTimeNs, x, y, psi)

	@classmethod
	def __addPolyToList(cls, poly: _InputPolyTypes, sensors: List[SensingPolygon], mapPolys: List[MovingPolygon | StaticPolygon]) -> None:
		if poly.type == SensingPolygon.type:
			sensors.append(poly)
		elif poly.type == StaticPolygon.type:
			pass
		elif poly.type == MovingPolygon.type:
			mapPolys.append(poly)
		else:
			raise TypeError("Unexpected region type: %s" % repr(poly.type))
		return

	@classmethod
	def __interpolateRegion(cls, oldPolys: SpatialRegion[_T_Poly], newPolys: SpatialRegion[_T_Poly], eventTimeNs: int, excludeRegions: List[MovingPolygon.Id], tracklets: List[Tracklet] = []) -> List[_T_Poly]:
		regions: List[_T_Poly] = []
		for id in oldPolys:
			if id in excludeRegions: continue
			if id not in newPolys.polygonIds: continue
			lastRegion = oldPolys[id]
			nowRegion = newPolys[id]
			ctRegion = ContinuousTimeRegion[_T_Poly]((lastRegion, nowRegion))
			poly = ctRegion[eventTimeNs]
			if poly.type == SensingPolygon.type:
				for track in tracklets: cast(SensingPolygon, poly).tracklets[track.id] = track
			regions.append(poly)
		return regions

	@classmethod
	def __obtainEventCGraphs(cls, sortedEventIntervals: List[CollisionInterval], lastCGraph: ConnectivityGraph) -> List[ConnectivityGraph]:
		if len(sortedEventIntervals) == 0: return []

		graphs: List[ConnectivityGraph] = []
		sensorPolys = [lastCGraph.sensors[i] for i in lastCGraph.sensors]
		mapPolys= [lastCGraph.map[i] for i in lastCGraph.map]
		for interval in sortedEventIntervals:
			(ctr1, _, ctr2, _, _, eventTimeNs) = interval
			# interpolatedTracks: List[Tracklet] = []
			# for tracklet in nowCGraph.fieldOfView.tracks.values():
			# 	previousTracklet = [tracklet for prevTracklet in lastCGraph.fieldOfView.tracks.values() if prevTracklet.id == tracklet.id]
			# 	if len(previousTracklet) == 1:
			# 		interpolatedTracks.append(cls.__interpolateTrack(previousTracklet[0], tracklet, eventTimeNs, startTime, endTime))
			# 	if len(previousTracklet) > 1: raise RuntimeError("How does this happen?")
			poly1 = ctr1[eventTimeNs]
			poly2 = ctr2[eventTimeNs]
			cls.__addPolyToList(poly1, sensorPolys, mapPolys)
			cls.__addPolyToList(poly2, sensorPolys, mapPolys)
			graph = ConnectivityGraph(timeNanoSecs=eventTimeNs, mapRegions=mapPolys, sensors=sensorPolys)
			graphs.append(graph)
		return graphs

	@classmethod
	def __aggregateIsomorphicGraphs(cls, graphs: List[ConnectivityGraph]) -> List[ConnectivityGraph]:
		filtered: List[ConnectivityGraph] = []
		for graph in graphs:
			if len(filtered) > 0 and cls.isIsomorphic(filtered[-1], graph): continue
			filtered.append(graph)
		return filtered

	@classmethod
	def aggregateCollisionEvents(cls, events: List[CollisionInterval], lastCGraph: ConnectivityGraph) -> List[ConnectivityGraph]:
		events.sort(key=lambda e: e[-1]) # Sort events by their end time
		Ros.Log("Aggregating for events @ %s" % repr([(e[4], e[5]) for e in events]))
		graphs: List[ConnectivityGraph] = cls.__obtainEventCGraphs(events, lastCGraph)
		graphs = cls.__aggregateIsomorphicGraphs(graphs)
		return graphs
