from typing import Any, Dict, List, Union

import rclpy
from sa_msgs.msg import RobotState
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_utils.RtBiEmulator import RtBiEmulator
from rt_bi_utils.RtBiNode import RtBiNode
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs


class DyRegTopicInterface(RtBiNode):
	""" This Node listens to all the messages published on the topics related to dynamic regions and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "rt_bi_core_dy", **kwArgs}
		super().__init__(**newKw)
		self.__regions: Union[Dict[int, Any], None] = None
		RtBiEmulator.subscribeToDyRegTopic(self, self.__onTargetUpdate)
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))

	def __onTargetUpdate(self, msg: RobotState) -> None:
		if msg is None:
			self.get_logger().warn("Received empty symbol!")
			return
		if (self.__regions is not None and msg.robot_id in self.__regions and hash(repr(msg)) == hash(repr(self.__regions[msg.robot_id]))): return

		self.get_logger().debug("Updating symbol %d definition." % msg.robot_id)
		if self.__regions is None:
			self.__regions = {}
		coords = SaMsgs.convertSaPoseListToCoordsList(msg.fov.corners)
		timeNanoSecs = self.get_clock().now().nanoseconds
		region = SymbolRegion(centerOfRotation=SaMsgs.convertSaPoseToPose(msg.pose), idNum=msg.robot_id, envelope=coords, timeNanoSecs=timeNanoSecs)
		self.__regions[msg.robot_id] = region
		self.render([region])
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseConfigFileParameters(self) -> None:
		return super().parseConfigFileParameters()

	def render(self, regions: List[Any]) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().info("Skipping map render... RViz is not ready yet to receive messages.")
			return
		message = MarkerArray()
		for region in regions:
			RosUtils.ConcatMessageArray(message.markers, region.render())
		self.__rvizPublisher.publish(message)
		return

def main(args=None):
	rclpy.init(args=args)
	node = DyRegTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
