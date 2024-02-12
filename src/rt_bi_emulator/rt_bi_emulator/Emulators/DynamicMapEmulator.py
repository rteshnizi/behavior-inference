import rclpy
from rclpy.logging import LoggingSeverity
from typing_extensions import cast

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Shared.TimeInterval import TimeInterval
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.MapRegion import MapRegion
from rt_bi_interfaces.msg import Polygon as RtBiPolygonMsg, RegularSpace, RegularSpaceArray, Traversability
from rt_bi_interfaces.srv import SpaceTime as SpaceTimeSvc


class DynamicMapEmulator(RtBiNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "em_dynamic_map", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.mapRegions: dict[str, MapRegion] = {}
		self.rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		self.__mapRegionsPublisher = RtBiInterfaces.createMapRegionsPublisher(self)
		Ros.WaitForServicesToStart(self, self.rdfClient)
		self.__coldStart()

	def __coldStart(self) -> None:
		req = SpaceTimeSvc.Request()
		req.filter.traversability.legs = Traversability.TRUE
		Ros.SendClientRequest(self, self.rdfClient, req, self.__onSpaceTimeResponse)
		return

	def __onSpaceTimeResponse(self, req: SpaceTimeSvc.Request, res: SpaceTimeSvc.Response) -> SpaceTimeSvc.Response:
		msg = RegularSpaceArray(spaces=res.spatial_matches)
		self.__mapRegionsPublisher.publish(msg)
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
