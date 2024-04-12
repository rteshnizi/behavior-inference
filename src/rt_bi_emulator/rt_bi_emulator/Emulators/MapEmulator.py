import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode, ColdStartPayload
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon


class MapEmulator(ColdStartableNode):
	"""
	This class listens to all static and dynamic map region updates:

	* Information about static regions are provided by a data source (e.g. an ontology or a static database).
	* Information about dynamic regions are provided by :class:`KnownRegionEmulator` instances.
	"""
	def __init__(self) -> None:
		newKw = { "node_name": "dynamic_map", "loggingSeverity": LoggingSeverity.WARN }
		super().__init__(**newKw)
		self.mapRegions: dict[str, StaticPolygon | MovingPolygon] = {}
		self.__mapRegionsPublisher = RtBiInterfaces.createMapRegionsPublisher(self)
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

	def __extractDynamicSetIds(self, matches: list[Msgs.RtBi.RegularSpace]) -> list[str]:
		dynamics = map(
			lambda m: m.id,
			filter(lambda m: m.space_type == Msgs.RtBi.RegularSpace.DYNAMIC, matches)
		)
		return list(dynamics)

	def __extractAffineSetIds(self, matches: list[Msgs.RtBi.RegularSpace]) -> list[str]:
		aff = map(
			lambda m: m.id,
			filter(lambda m: m.space_type == Msgs.RtBi.RegularSpace.AFFINE, matches)
		)
		return list(aff)

	def __onSpaceTimeResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
		msg = Msgs.RtBi.RegularSpaceArray(spaces=res.spatial_matches)
		payload = ColdStartPayload(req.json_payload)
		self.__mapRegionsPublisher.publish(msg)
		responsePayload = ColdStartPayload({
			"nodeName": self.get_fully_qualified_name(),
			"done": True,
			"phase": payload.phase,
			"affine": self.__extractAffineSetIds(Ros.AsList(res.spatial_matches, Msgs.RtBi.RegularSpace)),
			"dynamic": self.__extractDynamicSetIds(Ros.AsList(res.spatial_matches, Msgs.RtBi.RegularSpace)),
		})
		self.coldStartCompleted(responsePayload)
		# Request information about dynamic sets from the ontology
		dySetReq = Msgs.RtBiSrv.SpaceTime.Request()
		dySetReq.json_payload = responsePayload.stringify()
		Ros.SendClientRequest(self, self.rdfClient, dySetReq, self.__onDynamicSetsResponse)
		return res

	def __onDynamicSetsResponse(self, req: Msgs.RtBiSrv.SpaceTime.Request, res: Msgs.RtBiSrv.SpaceTime.Response) -> Msgs.RtBiSrv.SpaceTime.Response:
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
