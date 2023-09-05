from typing import Dict, List, Union

import rclpy
from rclpy.node import Node
from sa_msgs.msg import RobotState
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs


class SensorTopicInterface(Node):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, subClass=False, **kwArgs):
		""" Create a Viewer ROS node. """
		newKw = { "node_name": "rt_bi_core_sensor", **kwArgs}
		super().__init__(**newKw)
		if subClass:
			self.get_logger().info("%s in sensor topic init." % self.get_fully_qualified_name())
		else:
			self.get_logger().info("%s is initializing." % self.get_fully_qualified_name())
			RosUtils.SetLogger(self.get_logger())
		self.__sensors: Union[Dict[int, SensorRegion], None] = None
		if subClass:
			self.get_logger().info("%s skipping creating publishers and subscribers." % self.get_fully_qualified_name())
		else:
			SaMsgs.subscribeToSaRobotStateTopic(self, self.__onRobotStateUpdate)
			(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))

	def __onRobotStateUpdate(self, update: RobotState) -> None:
		if update is None:
			self.get_logger().warn("Received empty RobotState!")
			return False
		if (
			self.__sensors is not None and
			update.robot_id in self.__sensors and
			hash(repr(update)) == hash(repr(self.__sensors[update.robot_id]))
		):
			return False

		self.get_logger().info("Updating sensor %d definition." % update.robot_id)
		if self.__sensors is None:
			self.__sensors = {}
		coords = SaMsgs.convertSaPoseListToCoordsList(update.fov.corners)
		sensor = SensorRegion(idNum=update.robot_id, envelope=coords)
		self.__sensors[update.robot_id] = sensor
		self.render([sensor])
		return

	def render(self, regions: List[SensorRegion] = None):
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().warn("Skipping map render... RViz is not ready yet to receive messages.")
			return
		if regions is None:
			self.get_logger().info("Rendering map.")
			regionList = self.regions.values()
		else:
			self.get_logger().info("Rendering regions %s." % repr([r.name for r in regions]))
			regionList = regions
		message = MarkerArray()
		for region in regionList:
			message.markers += region.render()
		self.get_logger().info("MarkerArray about to be sent with %d markers." % len(message.markers))
		self.__rvizPublisher.publish(message)
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = SensorTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
