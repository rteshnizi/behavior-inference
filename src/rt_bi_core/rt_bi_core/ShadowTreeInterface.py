from typing import List

import rclpy
from rclpy.node import Client
from sa_msgs.msg import RobotState
from sa_msgs.srv import QueryFeature
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core import MapServiceInterface
from rt_bi_core.Eventifier.ShadowTree import ShadowTree
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs


class ShadowTreeInterface(MapServiceInterface):
	"""
	This Node listens to all the messages published on the topics related to the Shadow Tree.
	This node combines topic listeners and service clients.
	"""

	def __init__(self, liveRender=False) -> None:
		""" Create a Shadow Tree Interface node. """
		super().__init__(node_name="rt_bi_core_st")
		self.get_logger().info("%s is initializing." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__liveRender = liveRender
		self.__shadowTree: ShadowTree = ShadowTree()
		""" Dictionary of sensor id to region. """
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("shadow_tree"))
		self.__mapClient = SaMsgs.createSaFeatureQueryClient(self)
		SaMsgs.subscribeToSaRobotStateTopic(self, self.__onRobotStateUpdate)
		self.__shadowTreeColdStart()

	def __shadowTreeColdStart(self) -> None:
		self.requestMap(self.__mapClient)
		# # start a timer to query the feature definitions every 5 secs
		# self.create_timer(5, self.__queryDefinitions)
		return

	def __onRobotStateUpdate(self, update: RobotState) -> None:
		if update is None:
			self.get_logger().warn("Received empty RobotState!")
			return False

		timeNanoSecs = float(self.get_clock().now().nanoseconds)
		self.get_logger().info("Updating sensor %d definition @%d." % (update.robot_id, timeNanoSecs))
		coords = SaMsgs.convertSaPoseListToCoordsList(update.fov.corners)
		pose = SaMsgs.convertSaPoseToPose(update.pose)
		region = SensorRegion(centerOfRotation=pose, idNum=update.robot_id, envelope=coords, timeNanoSecs=timeNanoSecs)
		self.__updateSensors([region])
		return

	def __updateMap(self, regions: List[MapRegion]) -> None:
		timeNanoSecs = float(self.get_clock().now().nanoseconds)
		self.get_logger().info("Update map in ShadowTree with %s" % repr(regions))
		self.__shadowTree.updateMap(timeNanoSecs, regions)
		if self.__liveRender: self.render()
		return

	def __updateSymbol(self, regions: List[SymbolRegion]) -> None:
		self.get_logger().error("__updateSymbol not implemented.")
		return

	def __updateSensors(self, regions: List[SensorRegion]) -> None:
		updateTime = float(self.get_clock().now().nanoseconds)
		self.get_logger().info("Update sensors in ShadowTree %s @ %d" % (repr(regions), updateTime))
		self.__shadowTree.updateSensors(timeNanoSecs=updateTime, regions=regions)
		if self.__liveRender: self.render()
		return

	def parsePolygonShapeList(self, request: QueryFeature.Request, response: QueryFeature.Response) -> List[MapRegion]:
		regions = super().parsePolygonShapeList(request, response, False)
		self.__updateMap(regions)
		return regions

	def render(self) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().warn("Skipping ShadowTree render... RViz is not ready yet to receive messages.")
			return
		self.get_logger().info("Preparing to render ShadowTree.")
		for cGraph in self.__shadowTree.history:
			for name in cGraph.fieldOfView:
				region = cGraph.fieldOfView[name]

	def requestMap(self, mapClient: Client) -> None:
		return super().requestMap(mapClient)

def main(args=None) -> None:
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = ShadowTreeInterface(liveRender=True)
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
