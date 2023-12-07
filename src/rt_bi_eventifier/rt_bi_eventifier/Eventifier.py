from typing import Dict, List, Union

import rclpy
from rclpy.node import Client, Publisher
from rclpy.parameter import Parameter
from sa_msgs.msg import RobotState
from sa_msgs.srv import QueryFeature

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.MapServiceInterface import MapServiceInterface
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_eventifier.Model.ShadowTree import ShadowTree
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
		super().__init__(node_name="rt_bi_core_st")
		self.get_logger().debug("%s is initializing." % self.get_fully_qualified_name())
		self.__declareParameters()
		self.__liveRender: bool = False
		self.__renderModules: List[ShadowTree.SUBMODULE_TYPES] = []
		self.__parseConfigFileParameters()
		RosUtils.SetLogger(self.get_logger())
		modulePublishers: Dict[ShadowTree.SUBMODULE_TYPES, Union[Publisher, None]] = {}
		for module in ShadowTree.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher
		self.__shadowTree: ShadowTree = ShadowTree(modulePublishers)

		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("eventifier_interface"))
		self.__mapClient = SaMsgs.createSaFeatureQueryClient(self)
		SaMsgs.subscribeToSaRobotStateTopic(self, self.__onRobotStateUpdate)
		self.__shadowTreeColdStart()

	def __declareParameters(self) -> None:
		self.get_logger().debug("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("liveRender", Parameter.Type.BOOL)
		self.declare_parameter("renderModules", Parameter.Type.STRING_ARRAY)
		return

	def __parseConfigFileParameters(self) -> None:
		self.get_logger().debug("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__liveRender = self.get_parameter("liveRender").get_parameter_value().bool_value
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in ShadowTree.SUBMODULES:
				self.__renderModules.append(module)
			else:
				self.get_logger().warn(
					"Unknown module name in config file %s for node %s" % (module, self.get_fully_qualified_name())
				)
		return

	def __shadowTreeColdStart(self) -> None:
		self.requestMap(self.__mapClient)
		# # start a timer to query the feature definitions every 5 secs
		# self.create_timer(5, self.__queryDefinitions)
		return

	def __onRobotStateUpdate(self, update: RobotState) -> None:
		if update is None:
			self.get_logger().warn("Received empty RobotState!")
			return

		timeNanoSecs = self.get_clock().now().nanoseconds
		self.get_logger().debug("Updating sensor %d definition @%d." % (update.robot_id, timeNanoSecs))
		coords = SaMsgs.convertSaPoseListToCoordsList(update.fov.corners)
		pose = SaMsgs.convertSaPoseToPose(update.pose)
		region = SensorRegion(centerOfRotation=pose, idNum=update.robot_id, envelope=coords, timeNanoSecs=timeNanoSecs)
		self.__updateSensors([region])
		return

	def __updateMap(self, regions: List[MapRegion]) -> None:
		timeNanoSecs = self.get_clock().now().nanoseconds
		self.get_logger().debug("Update map in ShadowTree with %s" % repr(regions))
		self.__shadowTree.updateMap(timeNanoSecs, regions)
		if self.__liveRender: self.render()
		return

	def __updateSymbol(self, regions: List[SymbolRegion]) -> None:
		self.get_logger().error("__updateSymbol not implemented.")
		return

	def __updateSensors(self, regions: List[SensorRegion]) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().debug("Update sensors in ShadowTree %s @ %d" % (repr(regions), updateTime))
		self.__shadowTree.updateSensors(timeNanoSecs=updateTime, regions=regions)
		if self.__liveRender: self.render()
		return

	def parsePolygonShapeList(self, request: QueryFeature.Request, response: QueryFeature.Response) -> List[MapRegion]:
		regions = super().parsePolygonShapeList(request, response, False)
		self.__updateMap(regions)
		return regions

	def render(self) -> None:
		return

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
