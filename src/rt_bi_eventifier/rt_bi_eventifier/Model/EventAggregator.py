from typing import List, TypeVar

from networkx.algorithms.isomorphism import is_isomorphic

import rt_bi_commons.Utils.Ros as RosUtils
from rt_bi_core.AffineRegion import AffineRegion
from rt_bi_core.MapRegion import MapRegion
from rt_bi_core.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.SensorRegion import SensorRegion
from rt_bi_core.SymbolRegion import SymbolRegion
from rt_bi_core.Tracklet import Tracklet
from rt_bi_eventifier.Model.ConnectivityGraph import ConnectivityGraph
from rt_bi_eventifier.Model.ContinuousTimeCollisionDetection import CollisionInterval
from rt_bi_eventifier.Model.ContinuousTimeRegion import ContinuousTimeRegion
from rt_bi_eventifier.Model.MapRegions import MapRegions

RegionTypeX = TypeVar("RegionTypeX", SensorRegion, SymbolRegion, MapRegion)
RegionTypeY = TypeVar("RegionTypeY", SensorRegion, SymbolRegion, MapRegion)


class EventAggregator:
	"""
		This class contains functions related to Event Aggregator.
		For details see dissertation.
		© Reza Teshnizi 2018-2023
	"""

	@classmethod
	def __nodeNameMatcher(cls, n1: ConnectivityGraph.NodeContent, n2: ConnectivityGraph.NodeContent) -> bool:
		n1Content = n1["region"]
		n2Content = n2["region"]
		if not isinstance(n1Content, AffineRegion): raise ValueError("No Region found.")
		if not isinstance(n2Content, AffineRegion): raise ValueError("No Region found.")
		return n1Content.name == n2Content.name

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
	def __addRegionToList(cls, region: RegionTypeX, fovRegions: List[SensorRegion], symbolRegions: List[SymbolRegion]) -> None:
		if region.regionType == SensorRegion.RegionType.SENSING:
			fovRegions.append(region) # type: ignore
		elif region.regionType == MapRegion.RegionType.MAP:
			pass
		elif region.regionType == SymbolRegion.RegionType.SYMBOL:
			symbolRegions.append(region) # type: ignore
		else:
			raise TypeError("Unexpected region type: %s" % repr(region.regionType))
		return

	@classmethod
	def __interpolateRegion(cls, lastRegions: RegularAffineRegion[RegionTypeX], nowRegions: RegularAffineRegion[RegionTypeX], eventTimeNs: int, excludeRegions: List[str], tracklets: List[Tracklet] = []) -> List[RegionTypeX]:
		regions: List[RegionTypeX] = []
		for rName in lastRegions:
			if rName in excludeRegions: continue
			if rName not in nowRegions.regionNames: continue
			lastRegion = lastRegions[rName]
			nowRegion = nowRegions[rName]
			ctRegion = ContinuousTimeRegion[RegionTypeX]((lastRegion, nowRegion), lastRegion.regionType)
			region = ctRegion[eventTimeNs]
			if region.regionType == SensorRegion.RegionType.SENSING:
				for track in tracklets: region.tracklets.append(track) # type: ignore
			regions.append(region)
		return regions

	@classmethod
	def __obtainEventCGraphs(cls, sortedEventIntervals: List[CollisionInterval[RegionTypeX, RegionTypeY]], lastCGraph: ConnectivityGraph, mapRegions: MapRegions) -> List[ConnectivityGraph]:
		if len(sortedEventIntervals) == 0: return []

		graphs: List[ConnectivityGraph] = []
		fovRegions = [lastCGraph.fieldOfView[i] for i in lastCGraph.fieldOfView]
		symbolRegions= [lastCGraph.symbols[i] for i in lastCGraph.symbols]
		for interval in sortedEventIntervals:
			(ctRegion1, _, ctRegion2, _, _, eventTimeNs) = interval
			# interpolatedTracks: List[Tracklet] = []
			# for tracklet in nowCGraph.fieldOfView.tracks.values():
			# 	previousTracklet = [tracklet for prevTracklet in lastCGraph.fieldOfView.tracks.values() if prevTracklet.id == tracklet.id]
			# 	if len(previousTracklet) == 1:
			# 		interpolatedTracks.append(cls.__interpolateTrack(previousTracklet[0], tracklet, eventTimeNs, startTime, endTime))
			# 	if len(previousTracklet) > 1: raise RuntimeError("How does this happen?")
			region1 = ctRegion1[eventTimeNs]
			region2 = ctRegion2[eventTimeNs]
			cls.__addRegionToList(region1, fovRegions, symbolRegions)
			cls.__addRegionToList(region2, fovRegions, symbolRegions)
			graph = ConnectivityGraph(timeNanoSecs=eventTimeNs, mapRegions=mapRegions, fovRegions=fovRegions, symbols=symbolRegions)
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
	def aggregateCollisionEvents(cls, events: List[CollisionInterval[RegionTypeX, RegionTypeY]], lastCGraph: ConnectivityGraph, mapRegions: MapRegions) -> List[ConnectivityGraph]:
		events.sort(key=lambda e: e[-1]) # Sort events by their end time
		RosUtils.Log("Aggregating for events @ %s" % repr([(e[4], e[5]) for e in events]))
		graphs: List[ConnectivityGraph] = cls.__obtainEventCGraphs(events, lastCGraph, mapRegions)
		graphs = cls.__aggregateIsomorphicGraphs(graphs)
		return graphs
