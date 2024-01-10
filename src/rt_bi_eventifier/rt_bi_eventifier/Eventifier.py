from functools import partial
from typing import Dict, List, Tuple, Union

import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Client, Publisher
from rclpy.parameter import Parameter
from sa_msgs.srv import QueryFeature

from rt_bi_core.MapServiceInterface import MapServiceInterface
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_eventifier.Model.ShadowTree import ShadowTree
from rt_bi_interfaces.msg import DynamicRegion, Events, Graph
from rt_bi_utils.MinQueue import MinQueue
from rt_bi_utils.Ros import CreatePublisher, CreateTimer, CreateTopicName, Timer, TimeToInt
from rt_bi_utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs


class Eventifier(MapServiceInterface):
	"""
	This Node listens to all the messages published on the topics related to the Shadow Tree.
	This node combines topic listeners and service clients.
	"""

	def __init__(self) -> None:
		"""
		Create a Shadow Tree Interface node.
		"""
		super().__init__(loggingSeverity=LoggingSeverity.INFO, node_name="rt_bi_eventifier")
		self.declareParameters()
		self.__liveRender: bool = False
		self.__renderModules: List[ShadowTree.SUBMODULE_TYPES] = []
		self.__topicPublishers: Dict[ShadowTree.TOPICS, Publisher] = {}
		self.parseConfigFileParameters()
		modulePublishers: Dict[ShadowTree.SUBMODULE_TYPES, Union[Publisher, None]] = {}
		for module in ShadowTree.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher
		(self.__topicPublishers["eventifier_graph"], _) = CreatePublisher(self, Graph, CreateTopicName("eventifier_graph"))
		(self.__topicPublishers["eventifier_event"], _) = CreatePublisher(self, Events, CreateTopicName("eventifier_event"))
		self.__shadowTree: ShadowTree = ShadowTree(self.__topicPublishers, modulePublishers)
		self.__eventPQueue: MinQueue[Tuple[SensorRegion.RegionType, DynamicRegion]] = MinQueue(key=self.__eventPqKey)
		self.__processingTimer: Union[Timer, None] = None

		self.__mapClient = SaMsgs.createSaFeatureQueryClient(self)
		RtBiInterfaces.subscribeToSensorTopic(self, partial(self.__onDynamicRegionUpdate, SensorRegion.RegionType.SENSING))
		RtBiInterfaces.subscribeToSymbolTopic(self, partial(self.__onDynamicRegionUpdate, SymbolRegion.RegionType.SYMBOL))
		self.__shadowTreeColdStart()

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("liveRender", Parameter.Type.BOOL)
		self.declare_parameter("renderModules", Parameter.Type.STRING_ARRAY)
		return

	def parseConfigFileParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		self.__liveRender = self.get_parameter("liveRender").get_parameter_value().bool_value
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in ShadowTree.SUBMODULES:
				self.__renderModules.append(module)
			else:
				self.get_logger().warn(f"Unknown module name in config file {module} for node {self.get_fully_qualified_name()}")
		return

	def __eventPqKey(self, val: Tuple[SensorRegion.RegionType, DynamicRegion]) -> float:
		(_, region) = val
		return float(TimeToInt(region.stamp))

	def __shadowTreeColdStart(self) -> None:
		self.requestMap(self.__mapClient)
		# # start a timer to query the feature definitions every 5 secs
		# self.create_timer(5, self.__queryDefinitions)
		return

	def __processDynamicRegionUpdate(self) -> None:
		if self.__processingTimer is not None: self.__processingTimer.destroy()

		sensors: List[SensorRegion] = []
		symbols: List[SymbolRegion] = []
		while not self.__eventPQueue.isEmpty:
			(regionType, update) = self.__eventPQueue.dequeue()
			timeNanoSecs = TimeToInt(update.stamp)
			self.log(f"Updating region type {regionType} id {update.id} definition @{timeNanoSecs}.")
			coords = RtBiInterfaces.fromStdPoints32ToCoordsList(update.region.points) # type: ignore
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
		self.__processingTimer = CreateTimer(self, self.__processDynamicRegionUpdate)
		return

	def __onDynamicRegionUpdate(self, regionType: SensorRegion.RegionType, update: DynamicRegion) -> None:
		self.__eventPQueue.enqueue((regionType, update))
		self.__processDynamicRegionUpdate()
		return

	def __updateMap(self, regions: List[MapRegion]) -> None:
		timeNanoSecs = self.get_clock().now().nanoseconds
		self.log(f"Update map region @{timeNanoSecs}")
		self.__shadowTree.updateMap(timeNanoSecs, regions)
		if self.__liveRender: self.render()
		return

	def __updateAffineRegions(self, sensors: List[SensorRegion], symbols: List[SymbolRegion]) -> None:
		self.__shadowTree.updateAffineRegions(sensors=sensors, symbols=symbols)
		if self.__liveRender: self.render()
		return

	def parsePolygonShapeList(self, request: QueryFeature.Request, response: QueryFeature.Response) -> List[MapRegion]:
		regions = super().parsePolygonShapeList(request, response, False)
		self.__updateMap(regions)
		return regions

	def requestMap(self, mapClient: Client) -> None:
		return super().requestMap(mapClient)

def main(args=None) -> None:
	rclpy.init(args=args)
	node = Eventifier()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
