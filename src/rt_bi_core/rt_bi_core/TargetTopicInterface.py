from typing import Dict, List, Union

import rclpy
from rclpy.node import Node
from sa_msgs.msg import EstimationMsg, RobotState
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.TargetRegion import TargetRegion
from rt_bi_utils.RtBiEmulator import RtBiEmulator
from rt_bi_utils.RViz import RgbaNames, RViz
from rt_bi_utils.SaMsgs import SaMsgs


class TargetTopicInterface(Node):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, subClass=False, **kwArgs):
		""" Create a Viewer ROS node. """
		newKw = { "node_name": "rt_bi_core_target", **kwArgs}
		super().__init__(**newKw)
		if subClass:
			self.get_logger().debug("%s in sensor topic init." % self.get_fully_qualified_name())
		else:
			self.get_logger().debug("%s is initializing." % self.get_fully_qualified_name())
			RosUtils.SetLogger(self.get_logger())
		self.__targets: Union[Dict[int, TargetRegion], None] = None
		if subClass:
			self.get_logger().debug("%s skipping creating publishers and subscribers." % self.get_fully_qualified_name())
		else:
			RtBiEmulator.subscribeToTargetTopic(self, self.__onTargetUpdate)
			(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))
			self.__renderColor = RgbaNames.ORANGE

	def __onTargetUpdate(self, msg: RobotState) -> None:
		if msg is None:
			self.get_logger().warn("Received empty target!")
			return
		if (self.__targets is not None and msg.robot_id in self.__targets and hash(repr(msg)) == hash(repr(self.__targets[msg.robot_id]))): return

		self.get_logger().debug("Updating target %d definition." % msg.robot_id)
		if self.__targets is None:
			self.__targets = {}
		coords = SaMsgs.convertSaPoseListToCoordsList(msg.fov.corners)
		timeNanoSecs = self.get_clock().now().nanoseconds
		target = TargetRegion(centerOfRotation=SaMsgs.convertSaPoseToPose(msg.pose), idNum=msg.robot_id, envelope=coords, envelopeColor=self.__renderColor, timeNanoSecs=timeNanoSecs)
		self.__targets[msg.robot_id] = target
		self.render([target])
		return

	def render(self, regions: List[TargetRegion]) -> None:
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
	node = TargetTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
