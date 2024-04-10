import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Publisher
from rclpy.parameter import Parameter

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import MapSubscriber, SensorSubscriber
from rt_bi_core.Spatial import MapPolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ShadowTree import ShadowTree


class Eventifier(MapSubscriber, SensorSubscriber):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "eventifier", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(pauseQueuingMsgs=True, **newKw)
		self.declareParameters()
		self.__renderModules: list[ShadowTree.SUBMODULE] = []
		self.parseParameters()
		modulePublishers: dict[ShadowTree.SUBMODULE, Publisher | None] = {}
		for module in ShadowTree.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher

		gPublisher = RtBiInterfaces.createEventGraphPublisher(self)
		ePublisher = RtBiInterfaces.createEventPublisher(self)
		self.__shadowTree: ShadowTree = ShadowTree((gPublisher, ePublisher), modulePublishers)

	def onPolygonUpdated(self, rType: MovingPolygon.Type | StaticPolygon.Type | SensingPolygon.Type, polygon: MapPolygon | SensingPolygon) -> None:
		self.log(f"Updating polygons of type {rType}.")
		self.__shadowTree.updatePolygon(polygon)
		self.__shadowTree.renderLatestCGraph()
		if self.shouldRender: self.render()
		return

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("renderModules", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in ShadowTree.SUBMODULES:
				self.__renderModules.append(module)
			else:
				self.log(f"Unknown module name in config file {module} for node {self.get_fully_qualified_name()}")
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		return markers

def main(args=None) -> None:
	rclpy.init(args=args)
	node = Eventifier()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
