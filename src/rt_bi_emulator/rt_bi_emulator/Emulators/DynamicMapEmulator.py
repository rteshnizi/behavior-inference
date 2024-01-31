import rclpy
from rclpy.logging import LoggingSeverity
from typing_extensions import cast

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.TimeInterval import TimeInterval
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.MapRegion import MapRegion
from rt_bi_interfaces.msg import MapRegion as MapRegionMsg, MapRegions as MapRegionsMsg
from rt_bi_interfaces.srv import StaticReachability


class DynamicMapEmulator(RtBiNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "em_dynamic_map", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.mapRegions: dict[str, MapRegion] = {}
		self.rdfClient = RtBiInterfaces.createRdfClient(self, "static_reachability")
		self.__mapRegionsPublisher = RtBiInterfaces.createMapRegionsPublisher(self)
		Ros.WaitForServicesToStart(self, self.rdfClient)
		self.__coldStart()

	def __coldStart(self) -> None:
		req = StaticReachability.Request()
		req.include_type.legs = True
		req.include_type.swim = False
		req.include_type.wheels = False
		Ros.SendClientRequest(self, self.rdfClient, req, self.__onStaticReachabilityResponse)
		return

	def __onStaticReachabilityResponse(self, req: StaticReachability.Request, res: StaticReachability.Response) -> StaticReachability.Response:
		mapRegionsMsg = MapRegionsMsg()
		for regionMsg in res.regions:
			regionMsg = cast(MapRegionMsg, regionMsg)
			region = MapRegion(
				idNum=Ros.RegisterRegionId(regionMsg.id),
				envelope=RtBiInterfaces.fromStdPoints32ToCoordsList(regionMsg.region.points),
				envelopeColor=ColorNames.fromString(regionMsg.spec.color),
				offIntervals=[TimeInterval.fromMsg(interval) for interval in regionMsg.spec.off_intervals]
			)
			self.mapRegions[region.name] = region
			Ros.AppendMessage(mapRegionsMsg.regions, regionMsg)
		self.__mapRegionsPublisher.publish(mapRegionsMsg)
		return res

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	node = DynamicMapEmulator()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
