from typing import Dict, List

from networkx.algorithms.isomorphism import is_isomorphic

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.Tracklet import Tracklet, Tracklets
from rt_bi_utils.Geometry import Geometry


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
		return n1Content.interior.equals(n2Content.interior)

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
	def __interpolateTrack(cls, previousTracks: Tracklets, currentTracks: Tracklets, eventTime: float, eventFraction: float) -> Tracklets:
		"""
			This function assumes members in previousTracks and currentTracks all have the same timestamp.
		"""
		if len(previousTracks) == 0 or len(currentTracks) == 0: return currentTracks
		if eventFraction == 0: return previousTracks
		if eventFraction == 1: return currentTracks
		interpolatedTracks = {}
		(currentTime, _) = next(iter(currentTracks))
		for (prevTime, trackId) in previousTracks:
			previousPose = previousTracks[(prevTime, trackId)].pose
			currentPose = currentTracks[(currentTime, trackId)].pose
			x = ((eventFraction * (currentPose.x - previousPose.x)) + previousPose.x)
			y = ((eventFraction * (currentPose.y - previousPose.y)) + previousPose.y)
			psi = ((eventFraction * (currentPose.psi - previousPose.psi)) + previousPose.psi)
			interpolatedTracks[(eventTime, trackId)] = Tracklet(trackId, eventTime, x, y, psi, isInterpolated=True)
		return interpolatedTracks

	@classmethod
	def __obtainEventCGraphs(cls, events: List[CtCd.Collision], pastCGraph: ConnectivityGraph, nowCGraph: ConnectivityGraph, symbols: Dict[str, Symbol]) -> List[ConnectivityGraph]:
		RosUtils.Logger().debug("Creating CGraphs for %d events." % len(events))
		graphs: List[ConnectivityGraph] = []
		startTime = pastCGraph.timeNanoSecs
		endTime = nowCGraph.timeNanoSecs
		for event in events:
			(intermediatePolygon, timeOfEvent, eventType, staticEdge) = event
			eventTimeNs = int((relativeTime * (endTime - startTime)) + startTime)
			interpolatedTracks = cls.__interpolateTrack(pastCGraph.fieldOfView.tracks, nowCGraph.fieldOfView.tracks, eventTimeNs, event[1])
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTimeNs}
			sensorPolys = Geometry.toGeometryList(intermediatePolygon)
			fovRegions = [SensorRegion(centerOfRotation, idNum, envelope=Geometry.getGeometryCoords(poly), timeNanoSecs=eventTimeNs, tracks=filteredTracks) for poly in sensorPolys]
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
	def aggregateCollisionEvents(cls, events: List[CtCd.Collision], pastCGraph: ConnectivityGraph, nowCGraph: ConnectivityGraph, symbols: Dict[str, Symbol]) -> List[ConnectivityGraph]:
		RosUtils.Logger().debug("Aggregating for %d events." % len(events))
		graphs: List[ConnectivityGraph] = cls.__obtainEventCGraphs(events, pastCGraph, nowCGraph, symbols)
		graphs.sort(key=lambda g: g.timeNanoSecs)
		graphs = cls.__aggregateIsomorphicGraphs(graphs)
		return graphs
