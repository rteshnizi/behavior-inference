
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Publisher
from rclpy.parameter import Parameter

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber
from rt_bi_core.Spatial import MapPolygon
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_eventifier.Model.IGraph import IGraph


class Eventifier(RegionsSubscriber):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "eventifier", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.mapInitPhasesRemaining = 2
		"""
		The first two updates to the map are the static and dynamic regions, so
		we hold off on other affine updates until it is done.
		"""
		self.declareParameters()
		self.__renderModules: list[IGraph.SUBMODULE] = []
		self.parseParameters()
		modulePublishers: dict[IGraph.SUBMODULE, Publisher | None] = {}
		for module in IGraph.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher

		gPublisher = RtBiInterfaces.createEventGraphPublisher(self)
		ePublisher = RtBiInterfaces.createEventPublisher(self)
		self.__iGraph: IGraph = IGraph((gPublisher, ePublisher), modulePublishers)
		RtBiInterfaces.subscribeToProjectiveMap(self, self.enqueueUpdate)
		RtBiInterfaces.subscribeToAffineMap(self, self.enqueueUpdate)
		RtBiInterfaces.subscribeToSensors(self, self.enqueueUpdate)

	def __onUpdate(self, polygon: MapPolygon | SensingPolygon) -> None:
		self.log(f"Update received for polygon of type {polygon.type.name}.")
		# While initializing the map, don't take affine updates
		if self.mapInitPhasesRemaining > 0:
			if polygon.type == AffinePolygon.type or polygon.type == SensingPolygon.type:
				return
		self.__iGraph.updatePolygon(polygon)
		self.__iGraph.renderLatestCGraph()
		if self.shouldRender: self.render()
		return

	def onMapUpdated(self, polygon: MapPolygon) -> None:
		if self.mapInitPhasesRemaining > 0: self.mapInitPhasesRemaining -= 1
		return self.__onUpdate(polygon)

	def onSensorUpdated(self, polygon: SensingPolygon) -> None:
		return self.__onUpdate(polygon)

	def onTargetUpdated(self, _) -> None:
		# Eventifier receives the target observations through sensor emulator
		return

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("renderModules", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in IGraph.SUBMODULES:
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
