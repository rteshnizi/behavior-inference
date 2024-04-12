import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode, ColdStartPayload
from rt_bi_commons.Shared.MinQueue import MinQueue
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
		self.__reachabilityTimeQueue: MinQueue[tuple[int, TimeInterval, str]] = MinQueue(key=lambda r: r[0])
		self.__mapPublisher = RtBiInterfaces.createMapPublisher(self)
		self.rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServiceToStart(self, self.rdfClient)
		self.waitForColdStartPermission(self.onColdStartAllowed)
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		req = Msgs.RtBiSrv.SpaceTime.Request()
		req.json_payload = ColdStartPayload({
			"nodeName": self.get_fully_qualified_name(),
			"phase": payload.phase,
			"predicates": list(payload.predicates),
		}).stringify()
		Ros.SendClientRequest(self, self.rdfClient, req, self.__onSpaceTimeResponse)
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

	def __onSpaceTimeResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		res.sets = Ros.AsList(res.sets, Msgs.RtBi.RegularSet)
		self.__extractOriginOfTime(res.sets)
		msg = Msgs.RtBi.RegularSetArray(sets=res.sets)
		payload = ColdStartPayload(req.json_payload)
		self.__mapPublisher.publish(msg)
		responsePayload = ColdStartPayload({
			"nodeName": self.get_fully_qualified_name(),
			"done": True,
			"phase": payload.phase,
			"affine": self.__extractAffineSetIds(res.sets),
			"dynamic": self.__extractDynamicSetIds(res.sets),
		})
		self.coldStartCompleted(responsePayload)
		# Request information about dynamic sets from the ontology
		dySetReq = Msgs.RtBiSrv.SpaceTime.Request()
		dySetReq.json_payload = responsePayload.stringify()
		Ros.SendClientRequest(self, self.rdfClient, dySetReq, self.__onDynamicSetsResponse)
		return res

	def __processReachabilityEvents(self) -> None:
		nowNanoSecs = self.get_clock().now().nanoseconds
		msgArr = Msgs.RtBi.RegularSetArray()
		while not self.__reachabilityTimeQueue.isEmpty:
			(relativeTime, interval, setId) = self.__reachabilityTimeQueue.dequeue()
			eventTime = relativeTime + self.__timeOriginNanoSecs
			msg = Msgs.RtBi.RegularSet()
			msg.id = setId
			msg.space_type = Msgs.RtBi.RegularSet.DYNAMIC
			p = Msgs.RtBi.Predicate(
				name="reachable",
				value=Msgs.RtBi.Predicate.TRUE if nowNanoSecs in interval else Msgs.RtBi.Predicate.FALSE,
			)
			Ros.AppendMessage(msg.predicates, p)
			msg.stamp = Msgs.toTimeMsg(eventTime)
			Ros.AppendMessage(msgArr.sets, msg)
		self.__mapPublisher.publish(msgArr)
		return

	def __onDynamicSetsResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		for i in range(len(res.sets)):
			match = Ros.GetMessage(res.sets, i, Msgs.RtBi.RegularSet)
			for intervalMsg in match.intervals:
				interval = TimeInterval.fromMsg(intervalMsg)
				self.__reachabilityTimeQueue.enqueue((interval.minNanoSecs, interval, match.id))
				self.__reachabilityTimeQueue.enqueue((interval.maxNanoSecs, interval, match.id))
		self.__processReachabilityEvents()
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
