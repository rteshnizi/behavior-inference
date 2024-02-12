from functools import partial

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Publisher
from rclpy.parameter import Parameter

from rt_bi_commons.Base.MapRegionsSubscriber import MapRegionsSubscriber
from rt_bi_commons.Shared.MinQueue import MinQueue
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.SensorRegion import SensorRegion
from rt_bi_core.SymbolRegion import SymbolRegion
from rt_bi_eventifier.Model.ShadowTree import ShadowTree
from rt_bi_interfaces.msg import DynamicRegion, Events, Graph


class Eventifier(MapRegionsSubscriber):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "eventifier", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
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
		(self.__topicPublishers["eventifier_graph"], _) = Ros.CreatePublisher(self, Graph, Ros.CreateTopicName("eventifier_graph"))
		(self.__topicPublishers["eventifier_event"], _) = Ros.CreatePublisher(self, Events, Ros.CreateTopicName("eventifier_event"))
		self.__shadowTree: ShadowTree = ShadowTree(self.__topicPublishers, modulePublishers)
		self.__eventPQueue: MinQueue[tuple[SensorRegion.RegionType, DynamicRegion]] = MinQueue(key=self.__eventPqKey)
		self.__processingTimer: Ros.Timer | None = None
		RtBiInterfaces.subscribeToSensor(self, partial(self.__onDynamicRegionUpdate, SensorRegion.RegionType.SENSING))
		RtBiInterfaces.subscribeToSymbol(self, partial(self.__onDynamicRegionUpdate, SymbolRegion.RegionType.SYMBOL))

	def __eventPqKey(self, val: tuple[SensorRegion.RegionType, DynamicRegion]) -> float:
		(_, region) = val
		return float(Ros.TimeToInt(region.stamp))

	def __processDynamicRegionUpdate(self) -> None:
		if self.__processingTimer is not None: self.__processingTimer.destroy()

		sensors: list[SensorRegion] = []
		symbols: list[SymbolRegion] = []
		while not self.__eventPQueue.isEmpty:
			(regionType, update) = self.__eventPQueue.dequeue()
			timeNanoSecs = Ros.TimeToInt(update.stamp)
			self.log(f"Updating region type {regionType} id {update.id} definition @{timeNanoSecs}.")
			coords = RtBiInterfaces.fromStdPoints32ToCoordsList(update.region.points)
			cor = RtBiInterfaces.fromStdPointToCoords(update.center_of_rotation)
			if regionType == SensorRegion.RegionType.SENSING:
				region = SensorRegion(centerOfRotation=cor, idNum=update.id, envelope=coords, timeNanoSecs=timeNanoSecs)
				sensors.append(region)
			elif regionType == SymbolRegion.RegionType.SYMBOL:
				region = SymbolRegion(centerOfRotation=cor, idNum=update.id, envelope=coords, timeNanoSecs=timeNanoSecs, overlappingRegionId=0, overlappingRegionType=SymbolRegion.RegionType.BASE)
				symbols.append(region)
			else:
				raise TypeError(f"Unexpected region type: {regionType}")
		self.__updateAffineRegions(sensors, symbols)
		self.__processingTimer = Ros.CreateTimer(self, self.__processDynamicRegionUpdate)
		return

	def __onDynamicRegionUpdate(self, regionType: SensorRegion.RegionType, update: DynamicRegion) -> None:
		self.__eventPQueue.enqueue((regionType, update))
		self.__processDynamicRegionUpdate()
		return

	def __updateAffineRegions(self, sensors: list[SensorRegion], symbols: list[SymbolRegion]) -> None:
		self.__shadowTree.updateAffineRegions(sensors=sensors, symbols=symbols)
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
				self.get_logger().warn(f"Unknown module name in config file {module} for node {self.get_fully_qualified_name()}")
		return

	def onMapUpdated(self) -> None:
		timeNanoSecs = self.get_clock().now().nanoseconds
		self.get_logger().warn(f"{repr(self.mapRegions)}")
		self.log(f"Update map region @{timeNanoSecs}")
		self.__shadowTree.updateMap(timeNanoSecs, self.mapRegions)
		if self.shouldRender: self.render()
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
