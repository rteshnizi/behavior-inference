from typing import Literal

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartable, ColdStartPayload
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Temporal.TimeInterval import TimeInterval


class MapEmulator(ColdStartable):
	"""
	This class listens to all static and dynamic map region updates:

	* Information about static regions are provided by a data source (e.g. an ontology or a static database).
	* Information about dynamic regions are provided by :class:`KnownRegionEmulator` instances.
	"""
	def __init__(self) -> None:
		newKw = { "node_name": "dynamic_map", "loggingSeverity": LoggingSeverity.INFO }
		RtBiNode.__init__(self, **newKw)
		ColdStartable.__init__(self)
		self.__timeOriginNanoSecs: int = -1
		self.__mapPublisher = RtBiInterfaces.createMapPublisher(self)
		self.__coldStartPayload: ColdStartPayload | None = None
		self.__predicatesPublisher = RtBiInterfaces.createPredicatesPublisher(self)
		self.__rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServiceToStart(self, self.__rdfClient)
		self.waitForColdStartPermission()
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		self.__coldStartPayload = payload
		reqSpatial = Msgs.RtBiSrv.SpaceTime.Request()
		reqSpatial.query_name = "spatial"
		reqSpatial.json_payload = ColdStartPayload({"spatialPredicates": list(self.__coldStartPayload.spatialPredicates)}).stringify()
		Ros.SendClientRequest(self, self.__rdfClient, reqSpatial, self.__onSpatialPredicatesResponse)
		return

	def __extractOriginOfTime(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		if self.__timeOriginNanoSecs < 0 and len(matches) > 0:
			self.__timeOriginNanoSecs = Msgs.toNanoSecs(matches[0].stamp)
		return

	def __publishPredicateSymbols(self, predicateSymMapJson: str, namespace: Literal["spatial", "temporal"]) -> None:
		self.log(f"Publishing {namespace} predicate symbols.")
		predicateSymMapJson = predicateSymMapJson.replace("?p_", "p_")
		msg = Msgs.Std.String(data=predicateSymMapJson)
		self.__predicatesPublisher.publish(msg)
		return

	def __addTimePointToDict(self, setId: str, interval: TimeInterval, predicates: list[str], setDict: dict[str, list[tuple[TimeInterval, list[str]]]]) -> dict[str, list[tuple[TimeInterval, list[str]]]]:
		if setId not in setDict: setDict[setId] = []
		setDict[setId].append((interval, predicates))
		return setDict

	def __prepareIntervalsForProcessing(self, setDict: dict[str, list[tuple[TimeInterval, list[str]]]]) -> dict[str, list[tuple[TimeInterval, list[str]]]]:
		""" Sort reachability intervals and turn the relative time values to absolute. """
		Ros.Log("Preparing reachability intervals for processing.")
		for setId in setDict:
			intervals = setDict[setId]
			intervals = list(sorted(intervals, key=lambda i: i[0].minNanoSecs))
			for (interval, predicates) in intervals:
				interval.maxNanoSecs += self.__timeOriginNanoSecs
				interval.minNanoSecs += self.__timeOriginNanoSecs
			setDict[setId] = intervals
		return setDict

	def __createTemporalPredicateUpdate(self, setId: str, eventTime: int, val: bool, predicateNames: list[str], setType: str) -> Msgs.RtBi.RegularSet:
		msg = Msgs.RtBi.RegularSet()
		msg.id = setId
		msg.stamp = Msgs.toTimeMsg(eventTime)
		msg.set_type = setType
		for predicate in predicateNames:
			p = Msgs.RtBi.Predicate(name=predicate, value=Msgs.RtBi.Predicate.TRUE if val else Msgs.RtBi.Predicate.FALSE)
			Ros.AppendMessage(msg.predicates, p)
		return msg

	def __evaluateTemporalPredicates(self, timeNanoSecs: int, temporalSets: dict[str, list[tuple[TimeInterval, list[str]]]], setType: str, msgArr: Msgs.RtBi.RegularSetArray) -> Msgs.RtBi.RegularSetArray:
		"""Evaluate the membership of the given time point with respect to all the given temporal sets."""
		Ros.Log("Evaluating the current value of temporal predicates.")
		predicateVal: dict[str, bool] = {}
		for setId in temporalSets:
			intervals = temporalSets[setId].copy()
			predicateVal[setId] = False
			# Sequentially move forward in the list until timeNanoSecs is in the interval, or
			# the next interval starts later which means the set is not reachable,
			# given that the list is sorted and does not have overlap between intervals <-- this is assumed, not verified
			predicates: list[str] = []
			for (interval, predicates) in intervals:
				if timeNanoSecs in interval:
					predicateVal[setId] = True
					break
				elif interval.minNanoSecs > timeNanoSecs:
					break # It's a future event
				elif interval.maxNanoSecs < timeNanoSecs:
					# It's a past event
					# Remove past intervals from the list
					temporalSets[setId].remove((interval, predicates))
			predicateUpdateMsg = self.__createTemporalPredicateUpdate(setId, timeNanoSecs, predicateVal[setId], predicates, setType)
			Ros.Log(f"Evaluated {predicates} @ {timeNanoSecs} for {setId} to {predicateVal[setId]}.")
			Ros.AppendMessage(msgArr.sets, predicateUpdateMsg)
			if predicateVal[setId] == True:
				(currentInterval, predicates) = temporalSets[setId].pop(0)
				nextPredicateUpdate = self.__createTemporalPredicateUpdate(setId, currentInterval.maxNanoSecs, False, predicates, setType)
				Ros.Log(f"Reachability state for {setId} changes to False @ {currentInterval.maxNanoSecs}.")
				Ros.AppendMessage(msgArr.sets, nextPredicateUpdate)
		return msgArr

	def __futureTemporalEvents(self, temporalSets: dict[str, list[tuple[TimeInterval, list[str]]]], setType: str, msgArr: Msgs.RtBi.RegularSetArray) -> Msgs.RtBi.RegularSetArray:
		Ros.Log("Evaluating the future values of temporal predicates.")
		for setId in temporalSets:
			intervals = temporalSets[setId]
			Ros.Log(f"Evaluating temporal events for {setId}", intervals)
			for (interval, predicates) in intervals:
				Ros.Log(f"Evaluated {predicates} @ {interval.minNanoSecs} for {setId} to True.")
				update = self.__createTemporalPredicateUpdate(setId, interval.minNanoSecs, True, predicates, setType)
				Ros.AppendMessage(msgArr.sets, update)
				Ros.Log(f"Evaluated {predicates} @ {interval.maxNanoSecs} for {setId} to False.")
				update = self.__createTemporalPredicateUpdate(setId, interval.maxNanoSecs, False, predicates, setType)
				Ros.AppendMessage(msgArr.sets, update)
		return msgArr

	def __onTemporalResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		self.log("Received TEMPORAL PREDICATES response.")
		self.__publishPredicateSymbols(res.json_predicate_symbols, "temporal")
		res.sets = Ros.AsList(res.sets, Msgs.RtBi.RegularSet)
		self.__extractOriginOfTime(res.sets)
		nowNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		setDict: dict[str, list[tuple[TimeInterval, list[str]]]] = {}
		for match in res.sets:
			match.predicates = Ros.AsList(match.predicates, Msgs.RtBi.Predicate)
			predicates = [p.name for p in match.predicates]
			self.log(f"Temporal predicates include: {predicates}")
			match.intervals = Ros.AsList(match.intervals, Msgs.RtBi.TimeInterval)
			for intervalMsg in match.intervals:
				interval = TimeInterval.fromMsg(intervalMsg)
				setDict = self.__addTimePointToDict(match.id, interval, predicates, setDict)
		setDict = self.__prepareIntervalsForProcessing(setDict)
		temporalEvents = Msgs.RtBi.RegularSetArray()
		setType = Msgs.RtBi.RegularSet.TEMPORAL
		temporalEvents = self.__evaluateTemporalPredicates(nowNanoSecs, setDict, setType, temporalEvents)
		temporalEvents = self.__futureTemporalEvents(setDict, setType, temporalEvents)
		if len(temporalEvents.sets) > 0: self.__mapPublisher.publish(temporalEvents)
		self.publishColdStartDone()
		return res

	def __extractSetIdsByType(self, matches: list[Msgs.RtBi.RegularSet], filterType: str) -> list[str]:
		extracted = map(
			lambda m: m.id,
			filter(lambda m: m.set_type == filterType, matches)
		)
		return list(extracted)

	def __extractDynamicSetIds(self, matches: list[Msgs.RtBi.RegularSet]) -> list[str]:
		return self.__extractSetIdsByType(matches, Msgs.RtBi.RegularSet.DYNAMIC)

	def __extractAffineSetIds(self, matches: list[Msgs.RtBi.RegularSet]) -> list[str]:
		return self.__extractSetIdsByType(matches, Msgs.RtBi.RegularSet.AFFINE)

	def __publishProjectiveMap(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		self.log("Publishing projective map.")
		msg = Msgs.RtBi.RegularSetArray(sets=matches)
		self.__mapPublisher.publish(msg)
		return

	def __onSpatialPredicatesResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		self.log("Received SPATIAL PREDICATES response.")
		res.sets = Ros.AsList(res.sets, Msgs.RtBi.RegularSet)
		self.__extractOriginOfTime(res.sets)
		self.__publishProjectiveMap(res.sets)
		self.__publishPredicateSymbols(res.json_predicate_symbols, "spatial")
		# Request information about dynamic sets from the ontology.
		req = Msgs.RtBiSrv.SpaceTime.Request()
		req.query_name = "dynamic"
		req.json_payload = ColdStartPayload({
			"affine": self.__extractAffineSetIds(res.sets),
			"dynamic": self.__extractDynamicSetIds(res.sets),
		}).stringify()
		Ros.SendClientRequest(self, self.__rdfClient, req, self.__onDynamicSetsResponse)
		return res

	def __onDynamicSetsResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		self.log("Received DYNAMIC REACH response.")
		nowNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		accessibilityInfo: dict[str, list[tuple[TimeInterval, list[str]]]] = {}
		for i in range(len(res.sets)):
			match = Ros.GetMessage(res.sets, i, Msgs.RtBi.RegularSet)
			for intervalMsg in match.intervals:
				interval = TimeInterval.fromMsg(intervalMsg)
				accessibilityInfo = self.__addTimePointToDict(match.id, interval, ["accessible"], accessibilityInfo)
		accessibilityInfo = self.__prepareIntervalsForProcessing(accessibilityInfo)
		reachabilityUpdates = Msgs.RtBi.RegularSetArray()
		setType = Msgs.RtBi.RegularSet.DYNAMIC
		reachabilityUpdates = self.__evaluateTemporalPredicates(nowNanoSecs, accessibilityInfo, setType, reachabilityUpdates)
		reachabilityUpdates = self.__futureTemporalEvents(accessibilityInfo, setType, reachabilityUpdates)
		if len(reachabilityUpdates.sets) > 0: self.__mapPublisher.publish(reachabilityUpdates)
		assert self.__coldStartPayload is not None, "self.__coldStartPayload is None"
		reqTemporal = Msgs.RtBiSrv.SpaceTime.Request()
		reqTemporal.query_name = "temporal"
		reqTemporal.json_payload = ColdStartPayload({"temporalPredicates": list(self.__coldStartPayload.temporalPredicates)}).stringify()
		Ros.SendClientRequest(self, self.__rdfClient, reqTemporal, self.__onTemporalResponse)
		return res

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

	def render(self) -> None:
		return

def main(args=None):
	rclpy.init(args=args)
	node = MapEmulator()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
