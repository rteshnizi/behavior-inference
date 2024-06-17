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
		newKw = { "node_name": "dynamic_map", "loggingSeverity": Ros.LoggingSeverity.INFO }
		RtBiNode.__init__(self, **newKw)
		ColdStartable.__init__(self)
		self.__timeOriginNanoSecs: int = -1
		self.__mapPublisher = RtBiInterfaces.createProjectiveMapPublisher(self)
		self.__coldStartPayload: ColdStartPayload | None = None
		self.__predicatesPublisher = RtBiInterfaces.createPredicatesPublisher(self)
		self.__rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServiceToStart(self, self.__rdfClient)
		self.waitForColdStartPermission()
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		self.__coldStartPayload = payload
		reqSpatial = Msgs.RtBiSrv.SpaceTime.Request()
		reqSpatial.query_name = "sets"
		reqSpatial.json_payload = ColdStartPayload({"predicates": list(self.__coldStartPayload.predicates)}).stringify()
		Ros.SendClientRequest(self, self.__rdfClient, reqSpatial, self.__onRegularSetsResponse)
		return

	def __extractOriginOfTime(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		if self.__timeOriginNanoSecs < 0 and len(matches) > 0:
			self.__timeOriginNanoSecs = Msgs.toNanoSecs(matches[0].stamp)
		return

	def __publishPredicateSymbols(self, predicateSymMapJson: str) -> None:
		self.log(f"Publishing predicate symbols.")
		predicateSymMapJson = predicateSymMapJson.replace("?p_", "p_")
		msg = Msgs.Std.String(data=predicateSymMapJson)
		self.__predicatesPublisher.publish(msg)
		return

	def __addTimePointToDict(self, setId: str, setType: str, interval: TimeInterval, predicates: list[str], setDict: dict[str, list[tuple[str, TimeInterval, list[str]]]]) -> dict[str, list[tuple[str, TimeInterval, list[str]]]]:
		if setId not in setDict: setDict[setId] = []
		setDict[setId].append((setType, interval, predicates))
		return setDict

	def __prepareIntervalsForProcessing(self, setDict: dict[str, list[tuple[str, TimeInterval, list[str]]]]) -> dict[str, list[tuple[str, TimeInterval, list[str]]]]:
		""" Sort reachability intervals and turn the relative time values to absolute. """
		Ros.Log("Preparing reachability intervals for processing.")
		for setId in setDict:
			intervals = setDict[setId]
			intervals = list(sorted(intervals, key=lambda i: i[1].minNanoSecs))
			for (setType, interval, predicates) in intervals:
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

	def __evaluateTemporalPredicates(self, timeNanoSecs: int, temporalSets: dict[str, list[tuple[str, TimeInterval, list[str]]]], msgArr: Msgs.RtBi.RegularSetArray) -> Msgs.RtBi.RegularSetArray:
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
			for (setType, interval, predicates) in intervals:
				if timeNanoSecs in interval:
					predicateVal[setId] = True
					break
				elif interval.minNanoSecs > timeNanoSecs:
					break # It's a future event
				elif interval.maxNanoSecs < timeNanoSecs:
					# It's a past event
					# Remove past intervals from the list
					temporalSets[setId].remove((setType, interval, predicates))
			predicateUpdateMsg = self.__createTemporalPredicateUpdate(setId, timeNanoSecs, predicateVal[setId], predicates, setType)
			Ros.Log(f"Evaluated {predicates} @ {timeNanoSecs} for {setId} type {setType} = {predicateVal[setId]}.")
			Ros.AppendMessage(msgArr.sets, predicateUpdateMsg)
			if predicateVal[setId] == True:
				(setType, currentInterval, predicates) = temporalSets[setId].pop(0)
				nextPredicateUpdate = self.__createTemporalPredicateUpdate(setId, currentInterval.maxNanoSecs, False, predicates, setType)
				Ros.Log(f"Reachability state for {setId} changes to False @ {currentInterval.maxNanoSecs}.")
				Ros.AppendMessage(msgArr.sets, nextPredicateUpdate)
		return msgArr

	def __futureTemporalEvents(self, temporalSets: dict[str, list[tuple[str, TimeInterval, list[str]]]], msgArr: Msgs.RtBi.RegularSetArray) -> Msgs.RtBi.RegularSetArray:
		Ros.Log("Evaluating the future values of temporal predicates.")
		for setId in temporalSets:
			intervals = temporalSets[setId]
			Ros.Log(f"Evaluating temporal events for {setId}", intervals)
			for (setType, interval, predicates) in intervals:
				Ros.Log(f"Evaluated {predicates} @ {interval.minNanoSecs} for {setId} to True.")
				update = self.__createTemporalPredicateUpdate(setId, interval.minNanoSecs, True, predicates, setType)
				Ros.AppendMessage(msgArr.sets, update)
				Ros.Log(f"Evaluated {predicates} @ {interval.maxNanoSecs} for {setId} to False.")
				update = self.__createTemporalPredicateUpdate(setId, interval.maxNanoSecs, False, predicates, setType)
				Ros.AppendMessage(msgArr.sets, update)
		return msgArr

	def __extractSetsByType(self, matches: list[Msgs.RtBi.RegularSet], filterType: list[str]) -> list[Msgs.RtBi.RegularSet]:
		# extracted = map(lambda m: m.id, filter(lambda m: m.set_type in filterType, matches)) # Maps sets to IDs
		extracted = filter(lambda m: (m.set_type in filterType), matches)
		return list(extracted)

	def __publishProjectiveMap(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		self.log("Publishing projective map.")
		matches = self.__extractSetsByType(matches, [
			Msgs.RtBi.RegularSet.STATIC,
			Msgs.RtBi.RegularSet.DYNAMIC
		])
		msg = Msgs.RtBi.RegularSetArray(sets=matches)
		self.__mapPublisher.publish(msg)
		return

	def __onRegularSetsResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		nowNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		self.log("Received REGULAR SETS response.")
		res.sets = Ros.AsList(res.sets, Msgs.RtBi.RegularSet)
		self.__extractOriginOfTime(res.sets)
		self.__publishProjectiveMap(res.sets)
		self.__publishPredicateSymbols(res.json_predicate_symbols)
		setsWithIntervals = self.__extractSetsByType(res.sets, [Msgs.RtBi.RegularSet.DYNAMIC, Msgs.RtBi.RegularSet.TEMPORAL])
		setDict: dict[str, list[tuple[str, TimeInterval, list[str]]]] = {}
		for match in setsWithIntervals:
			match.predicates = Ros.AsList(match.predicates, Msgs.RtBi.Predicate)
			predicates = [p.name for p in match.predicates]
			self.log(f"Temporal predicates of {match.id} include: {predicates}")
			match.intervals = Ros.AsList(match.intervals, Msgs.RtBi.TimeInterval)
			for intervalMsg in match.intervals:
				interval = TimeInterval.fromMsg(intervalMsg)
				setDict = self.__addTimePointToDict(match.id, match.set_type, interval, predicates, setDict)
		setDict = self.__prepareIntervalsForProcessing(setDict)
		temporalEvents = Msgs.RtBi.RegularSetArray()
		temporalEvents = self.__evaluateTemporalPredicates(nowNanoSecs, setDict, temporalEvents)
		temporalEvents = self.__futureTemporalEvents(setDict, temporalEvents)
		if len(temporalEvents.sets) > 0: self.__mapPublisher.publish(temporalEvents)
		self.publishColdStartDone()
		return res

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

	def render(self) -> None:
		return

def main(args=None) -> None:
	return MapEmulator.Main(args)

if __name__ == "__main__":
	main()
