from typing import Dict, List, TypeVar

from networkx.algorithms.isomorphism import categorical_node_match, is_isomorphic

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import CollisionInterval, ContinuousTimeCollisionDetection as CtCd
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.Tracklet import Tracklet, Tracklets
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Pose import Pose

RegionTypeX = TypeVar("RegionTypeX", bound=DynamicRegion)
RegionTypeY = TypeVar("RegionTypeY", bound=DynamicRegion)


class EventAggregator:
	"""
		This class contains functions related to Event Aggregator.
		For details see dissertation.
		© Reza Teshnizi 2018-2023
	"""

	@classmethod
	def __nodeInteriorMatcher(cls, n1: ConnectivityGraph.NodeContent, n2: ConnectivityGraph.NodeContent) -> bool:
		n1Content = n1["region"]
		n2Content = n2["region"]
		if not isinstance(n1Content, DynamicRegion): raise ValueError("No Region found.")
		if not isinstance(n2Content, DynamicRegion): raise ValueError("No Region found.")
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
		if is_isomorphic(g1, g2, node_match= cls.__nodeInteriorMatcher if useMatcher else None):
			return True
		else:
			return False

	@classmethod
	def __interpolateTrack(cls, previousTracks: Tracklets, currentTracks: Tracklets, eventTimeNs: int, startTimeNs: int, endTimeNs: int) -> Tracklets:
		"""
			This function assumes members in previousTracks and currentTracks all have the same timestamp.
		"""
		if len(previousTracks) == 0 or len(currentTracks) == 0: return currentTracks
		if eventTimeNs == startTimeNs: return previousTracks
		if eventTimeNs == endTimeNs: return currentTracks
		interpolatedTracks = {}
		(currentTime, _) = next(iter(currentTracks))
		eventFraction = int((eventTimeNs - startTimeNs) / (endTimeNs - startTimeNs))
		for (prevTime, trackId) in previousTracks:
			previousPose = previousTracks[(prevTime, trackId)].pose
			currentPose = currentTracks[(currentTime, trackId)].pose
			x = ((eventFraction * (currentPose.x - previousPose.x)) + previousPose.x)
			y = ((eventFraction * (currentPose.y - previousPose.y)) + previousPose.y)
			psi = ((eventFraction * (currentPose.psi - previousPose.psi)) + previousPose.psi)
			interpolatedTracks[(eventTimeNs, trackId)] = Tracklet(trackId, eventTimeNs, x, y, psi, isInterpolated=True)
		return interpolatedTracks

	@classmethod
	def __obtainEventCGraphs(cls, eventIntervals: List[CollisionInterval[RegionTypeX, RegionTypeY]], pastCGraph: ConnectivityGraph, nowCGraph: ConnectivityGraph, symbols: Dict[str, Symbol]) -> List[ConnectivityGraph]:
		RosUtils.Logger().debug("Creating CGraphs for %d events." % len(eventIntervals))
		graphs: List[ConnectivityGraph] = []
		startTime = pastCGraph.timeNanoSecs
		endTime = nowCGraph.timeNanoSecs
		for interval in eventIntervals:
			(ctRegion1, _, ctRegion2, _, _, eventTimeNs) = interval
			interpolatedTracks = cls.__interpolateTrack(pastCGraph.fieldOfView.tracks, nowCGraph.fieldOfView.tracks, eventTimeNs, startTime, endTime)
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTimeNs}
			fovRegions: List[SensorRegion] = []
			if ctRegion1.regionType == DynamicRegion.RegionType.SENSING:
				poly = ctRegion1[eventTimeNs]
				fovRegions.append(SensorRegion(Pose(0, 0, 0, 0), ctRegion1.idNum, envelope=Geometry.getGeometryCoords(poly), timeNanoSecs=eventTimeNs, tracks=filteredTracks))
			if ctRegion2.regionType == DynamicRegion.RegionType.SENSING:
				poly = ctRegion1[eventTimeNs]
				fovRegions.append(SensorRegion(Pose(0, 0, 0, 0), ctRegion1.idNum, envelope=Geometry.getGeometryCoords(poly), timeNanoSecs=eventTimeNs, tracks=filteredTracks))
			graph = ConnectivityGraph(timeNanoSecs=eventTimeNs, mapRegions=nowCGraph.mapPerimeter, fovRegions=fovRegions)
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
	def aggregateCollisionEvents(cls, events: List[CollisionInterval], pastCGraph: ConnectivityGraph, nowCGraph: ConnectivityGraph, symbols: Dict[str, Symbol]) -> List[ConnectivityGraph]:
		RosUtils.Logger().debug("Aggregating for %d events." % len(events))
		graphs: List[ConnectivityGraph] = cls.__obtainEventCGraphs(events, pastCGraph, nowCGraph, symbols)
		graphs.sort(key=lambda g: g.timeNanoSecs)
		graphs = cls.__aggregateIsomorphicGraphs(graphs)
		return graphs
