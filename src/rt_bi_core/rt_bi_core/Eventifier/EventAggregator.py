from typing import Dict, List

from networkx.algorithms.isomorphism import is_isomorphic

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ContinuousTimeCollisionDetection import ContinuousTimeCollisionDetection as CtCd
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
		return n1["region"].interior.equals(n2["region"].interior)

	@classmethod
	def __isIsomorphic(cls, g1: ConnectivityGraph, g2: ConnectivityGraph) -> bool:
		if is_isomorphic(g1, g2):
			RosUtils.Logger().warn("Isomorphic graph rejected in event aggregation.")
			return True
		else:
			RosUtils.Logger().error("N: %s, E: %s" % (repr({n for n in g2.nodes}), repr({e for e in g2.edges})))
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
	def __aggregateConnectivityGraphs(cls, events: List[CtCd.CollisionEvent], pastCGraph: ConnectivityGraph, nowCGraph: ConnectivityGraph, symbols: Dict[str, Symbol]) -> List[ConnectivityGraph]:
		graphs = []
		startTime = pastCGraph.timeNanoSecs
		endTime = nowCGraph.timeNanoSecs
		for event in events:
			(intermediatePolygon, idNum, centerOfRotation, relativeTime, eventType, staticEdge) = event
			eventTimeNs = int((relativeTime * (endTime - startTime)) + startTime)
			interpolatedTracks = cls.__interpolateTrack(pastCGraph.fieldOfView.tracks, nowCGraph.fieldOfView.tracks, eventTimeNs, event[1])
			filteredTracks = {(tTime, tId): interpolatedTracks[(tTime, tId)] for (tTime, tId) in interpolatedTracks if tTime == eventTimeNs}
			sensorPolys = Geometry.toGeometryList(intermediatePolygon)
			fovRegions = [SensorRegion(centerOfRotation, idNum, envelope=Geometry.getPolygonCoords(poly), timeNanoSecs=eventTimeNs, tracks=filteredTracks) for poly in sensorPolys]
			graph = ConnectivityGraph(timeNanoSecs=eventTimeNs, mapRegions=nowCGraph.mapPerimeter, fovRegions=fovRegions)
			if len(graphs) > 0 and cls.__isIsomorphic(graphs[-1], graph): continue
			graphs.append(graph)
		return graphs

	@classmethod
	def obtainCollisionCGraphs(self, events: List[CtCd.CollisionEvent], pastCGraph: ConnectivityGraph, nowCGraph: ConnectivityGraph, symbols: Dict[str, Symbol]) -> List[ConnectivityGraph]:
		RosUtils.Logger().info("Creating CGraphs for %d events." % len(events))
		eventGraphs = self.__aggregateConnectivityGraphs(events=events, pastCGraph=pastCGraph, nowCGraph=nowCGraph, symbols=symbols)
		return eventGraphs
