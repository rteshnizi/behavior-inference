import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode, ColdStartPayload
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Temporal.TimeInterval import TimeInterval


class MapEmulator(ColdStartableNode):
	"""
	This class listens to all static and dynamic map region updates:

	* Information about static regions are provided by a data source (e.g. an ontology or a static database).
	* Information about dynamic regions are provided by :class:`KnownRegionEmulator` instances.
	"""
	def __init__(self) -> None:
		newKw = { "node_name": "dynamic_map", "loggingSeverity": LoggingSeverity.WARN }
		super().__init__(**newKw)
		self.__timeOriginNanoSecs: int = -1
		self.__reachabilityInformation: dict[str, list[TimeInterval]] = {}
		self.__reachabilityState: dict[str, bool] = {}
		self.__mapPublisher = RtBiInterfaces.createMapPublisher(self)
		self.__predicatesPublisher = RtBiInterfaces.createPredicatesPublisher(self)
		self.rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServiceToStart(self, self.rdfClient)
		self.waitForColdStartPermission(self.onColdStartAllowed)
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		req = Msgs.RtBiSrv.SpaceTime.Request()
		req.query_name = "spatial"
		req.json_payload = ColdStartPayload({
			"nodeName": self.get_fully_qualified_name(),
			"predicates": list(payload.predicates),
		}).stringify()
		Ros.SendClientRequest(self, self.rdfClient, req, self.__onStaticReachabilityResponse)
		return

	def __extractSetIdsByType(self, matches: list[Msgs.RtBi.RegularSet], filterType: str) -> list[str]:
		extracted = map(
			lambda m: m.id,
			filter(lambda m: m.space_type == filterType, matches)
		)
		return list(extracted)

	def __extractDynamicSetIds(self, matches: list[Msgs.RtBi.RegularSet]) -> list[str]:
		return self.__extractSetIdsByType(matches, Msgs.RtBi.RegularSet.DYNAMIC)

	def __extractAffineSetIds(self, matches: list[Msgs.RtBi.RegularSet]) -> list[str]:
		return self.__extractSetIdsByType(matches, Msgs.RtBi.RegularSet.AFFINE)

	def __extractOriginOfTime(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		if self.__timeOriginNanoSecs < 0 and len(matches) > 0:
			self.__timeOriginNanoSecs = Msgs.toNanoSecs(matches[0].stamp)
		return

	def __publishProjectiveMap(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		msg = Msgs.RtBi.RegularSetArray(sets=matches)
		self.__mapPublisher.publish(msg)
		return

	def __publishPredicateSymbols(self, predicateSymMapJson: str) -> None:
		self.log(f"PREDICATES = {predicateSymMapJson}")
		predicateSymMapJson = predicateSymMapJson.replace("?p_", "p_")
		msg = Msgs.Std.String(data=predicateSymMapJson)
		self.__predicatesPublisher.publish(msg)
		return

	def __queryDynamicReach(self, matches: list[Msgs.RtBi.RegularSet]) -> None:
		"""Request information about dynamic sets from the ontology."""
		req = Msgs.RtBiSrv.SpaceTime.Request()
		req.query_name = "dynamic"
		req.json_payload = ColdStartPayload({
			"nodeName": self.get_fully_qualified_name(),
			"affine": self.__extractAffineSetIds(matches),
			"dynamic": self.__extractDynamicSetIds(matches),
		}).stringify()
		Ros.SendClientRequest(self, self.rdfClient, req, self.__onDynamicSetsResponse)
		return

	def __onStaticReachabilityResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		self.log("Received STATIC REACH response.")
		res.sets = Ros.AsList(res.sets, Msgs.RtBi.RegularSet)
		self.__extractOriginOfTime(res.sets)
		self.__publishProjectiveMap(res.sets)
		self.__publishPredicateSymbols(res.json_predicate_symbols)
		self.__queryDynamicReach(res.sets)
		responsePayload = ColdStartPayload({
			"nodeName": self.get_fully_qualified_name(),
			"done": True,
		})
		self.coldStartCompleted(responsePayload)
		return res

	def __createReachabilityUpdate(self, setId: str, eventTime: int, reachable: bool) -> Msgs.RtBi.RegularSet:
		msg = Msgs.RtBi.RegularSet()
		msg.id = setId
		msg.space_type = Msgs.RtBi.RegularSet.DYNAMIC
		p = Msgs.RtBi.Predicate(name="accessible", value=Msgs.RtBi.Predicate.TRUE if reachable else Msgs.RtBi.Predicate.FALSE)
		Ros.AppendMessage(msg.predicates, p)
		msg.stamp = Msgs.toTimeMsg(eventTime)
		return msg

	def __prepareIntervalsForProcessing(self) -> None:
		""" Sort reachability intervals and turn the relative time values to absolute. """
		Ros.Log("Preparing reachability intervals for processing.")
		for setId in self.__reachabilityInformation:
			intervals = self.__reachabilityInformation[setId]
			intervals = list(sorted(intervals, key=lambda i: i.minNanoSecs))
			for interval in intervals:
				interval.maxNanoSecs += self.__timeOriginNanoSecs
				interval.minNanoSecs += self.__timeOriginNanoSecs
			self.__reachabilityInformation[setId] = intervals
		return

	def __determineCurrentReachabilityState(self, nowNanoSecs: int, msgArr: Msgs.RtBi.RegularSetArray) -> Msgs.RtBi.RegularSetArray:
		Ros.Log(f"Processing current reachability states.")
		for setId in self.__reachabilityInformation:
			intervals = self.__reachabilityInformation[setId].copy()
			self.__reachabilityState[setId] = False
			# Sequentially move forward in the list until now is in the interval, or
			# the next interval starts later which means the set is not reachable since the list is sorted
			# and does not have overlap between intervals <-- this is assumed, not verified
			for interval in intervals:
				if nowNanoSecs in interval:
					self.__reachabilityState[setId] = True
					break
				elif interval.minNanoSecs > nowNanoSecs:
					break # It's a future event
				elif interval.maxNanoSecs < nowNanoSecs:
					# It's a past event
					# Remove past intervals from the list
					self.__reachabilityInformation[setId].remove(interval)
			currentReachability = self.__createReachabilityUpdate(setId, nowNanoSecs, self.__reachabilityState[setId])
			Ros.Log(f"Current reachability states for {setId} is {self.__reachabilityState[setId]}.")
			Ros.AppendMessage(msgArr.sets, currentReachability)
			if self.__reachabilityState[setId] == True:
				currentInterval = self.__reachabilityInformation[setId].pop(0)
				nextReachability = self.__createReachabilityUpdate(setId, currentInterval.maxNanoSecs, False)
				Ros.Log(f"Reachability state for {setId} changes to False @ {currentInterval.maxNanoSecs}.")
				Ros.AppendMessage(msgArr.sets, nextReachability)
		return msgArr

	def __futureReachabilityEvents(self, msgArr: Msgs.RtBi.RegularSetArray) -> Msgs.RtBi.RegularSetArray:
		Ros.Log(f"Processing future reachability states.")
		for setId in self.__reachabilityInformation:
			intervals = self.__reachabilityInformation[setId]
			Ros.Log(f"Processing reachability events for {setId}", intervals)
			for interval in intervals:
				update = self.__createReachabilityUpdate(setId, interval.minNanoSecs, True)
				Ros.AppendMessage(msgArr.sets, update)
				update = self.__createReachabilityUpdate(setId, interval.maxNanoSecs, False)
				Ros.AppendMessage(msgArr.sets, update)
		return msgArr

	def __processReachabilityUpdates(self, nowNanoSecs: int) -> Msgs.RtBi.RegularSetArray:
		reachabilityUpdates = Msgs.RtBi.RegularSetArray()
		self.__prepareIntervalsForProcessing()
		reachabilityUpdates = self.__determineCurrentReachabilityState(nowNanoSecs, reachabilityUpdates)
		reachabilityUpdates = self.__futureReachabilityEvents(reachabilityUpdates)
		return reachabilityUpdates

	def __addTimePointToDict(self, setId: str, interval: TimeInterval) -> None:
		if setId not in self.__reachabilityInformation: self.__reachabilityInformation[setId] = []
		self.__reachabilityInformation[setId].append(interval)
		return

	def __onDynamicSetsResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		self.log("Received DYNAMIC REACH response.")
		nowNanoSecs = Msgs.toNanoSecs(self.get_clock().now())
		for i in range(len(res.sets)):
			match = Ros.GetMessage(res.sets, i, Msgs.RtBi.RegularSet)
			for intervalMsg in match.intervals:
				interval = TimeInterval.fromMsg(intervalMsg)
				self.__addTimePointToDict(match.id, interval)
		reachabilityUpdates = self.__processReachabilityUpdates(nowNanoSecs)
		if len(reachabilityUpdates.sets) > 0: self.__mapPublisher.publish(reachabilityUpdates)
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
