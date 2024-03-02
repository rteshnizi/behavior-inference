from itertools import chain

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Publisher
from rclpy.parameter import Parameter

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import MapSubscriber, SensorSubscriber
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_eventifier.Model.ShadowTree import ShadowTree


class Eventifier(MapSubscriber, SensorSubscriber):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "eventifier", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.__renderModules: list[ShadowTree.SUBMODULE_TYPES] = []
		self.__topicPublishers: dict[ShadowTree.TOPICS, Publisher] = {}
		self.parseParameters()
		modulePublishers: dict[ShadowTree.SUBMODULE_TYPES, Publisher | None] = {}
		for module in ShadowTree.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher
		(self.__topicPublishers["eventifier_graph"], _) = Ros.CreatePublisher(self, Msgs.RtBi.Graph, Ros.CreateTopicName("eventifier_graph"))
		(self.__topicPublishers["eventifier_event"], _) = Ros.CreatePublisher(self, Msgs.RtBi.Events, Ros.CreateTopicName("eventifier_event"))
		self.__shadowTree: ShadowTree = ShadowTree(self.__topicPublishers, modulePublishers)

	def onRegionsUpdated(self, rType: MovingPolygon.Types, regions: list[Msgs.RtBi.RegularSpace]) -> None:
		self.log(f"Updating polygons of type {rType}.")
		for msg in regions:
			match rType:
				case StaticPolygon.type | MovingPolygon.type:
					region = self.mapRegions[msg.id]
				case SensingPolygon.type:
					region = self.sensors[msg.id]
				case _:
					raise TypeError(f"Unexpected region type: {rType}")
			self.__shadowTree.updateRegion(region)
			if self.shouldRender: self.render()
		return

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("renderModules", Parameter.Type.STRING_ARRAY)
		return
 # pyright: ignore[reportArgumentType]
	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in ShadowTree.SUBMODULES:
				self.__renderModules.append(module)
			else:
				self.log(f"Unknown module name in config file {module} for node {self.get_fully_qualified_name()}")
		return

def main(args=None) -> None:
	rclpy.init(args=args)
	node = Eventifier()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
