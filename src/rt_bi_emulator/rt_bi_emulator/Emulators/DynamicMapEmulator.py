
import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode, ColdStartPayload
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.MapRegion import MapRegion
from rt_bi_interfaces.msg import RegularSpaceArray
from rt_bi_interfaces.srv import SpaceTime


class DynamicMapEmulator(ColdStartableNode):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "em_dynamic_map", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)
		self.mapRegions: dict[str, MapRegion] = {}
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
		# for m in res.spatial_matches:
		# 	m = cast(RegularSpaceMsg, m)
		# 	self.log(f"RESPONSE ID = {repr(m.id)}")
		# 	for p in m.predicates:
		# 		p = cast(Predicate, p)
		# 		self.log(f"RESPONSE PRED = {repr((p.name, p.value))}")
		# 	for p in m.polygons:
		# 		p = cast(RtBiPolygonMsg, p)
		# 		self.log(f"RESPONSE POLY = {repr((p.id, [(pt.x, pt.y) for pt in p.region.points]))}")

		msg = RegularSpaceArray(spaces=res.spatial_matches)
		self.__mapRegionsPublisher.publish(msg)
		self.coldStartCompleted({
			"done": True,
		})
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
