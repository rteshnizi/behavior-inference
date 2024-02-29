import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode, ColdStartPayload
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Polygons.StaticPolygon import StaticPolygon
from rt_bi_interfaces.msg import RegularSpaceArray
from rt_bi_interfaces.srv import SpaceTime


class MapEmulator(ColdStartableNode):
	"""
	This class listens to all static and dynamic map region updates:

	* Information about static regions are provided by a data source (e.g. an ontology or a static database).
	* Information about dynamic regions are provided by :class:`MapRegionEmulator` instances.
	"""
	def __init__(self) -> None:
		newKw = { "node_name": "emulator_dynamic_map", "loggingSeverity": LoggingSeverity.WARN }
		super().__init__(**newKw)
		self.mapRegions: dict[str, StaticPolygon] = {}
		self.__mapRegionsPublisher = RtBiInterfaces.createMapRegionsPublisher(self)
		self.rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServicesToStart(self, self.rdfClient)
		self.waitForColdStartPermission(self.onColdStartAllowed)
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		req = SpaceTime.Request()
		req.json_payload = payload.stringify()
		Ros.SendClientRequest(self, self.rdfClient, req, self.__onSpaceTimeResponse)
		return

	def __onSpaceTimeResponse(self, req: SpaceTime.Request, res: SpaceTime.Response) -> SpaceTime.Response:
		msg = RegularSpaceArray(spaces=res.spatial_matches)
		self.__mapRegionsPublisher.publish(msg)
		self.coldStartCompleted({ "done": True })
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
