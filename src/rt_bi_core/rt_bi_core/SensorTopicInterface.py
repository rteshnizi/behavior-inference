from functools import partial
from typing import Dict, List, Union

import rclpy
from rclpy.node import Node
from sa_msgs.msg import EstimationMsg, RobotState
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.Tracklet import Tracklet
from rt_bi_utils.RViz import RViz
from rt_bi_utils.SaMsgs import SaMsgs


class SensorTopicInterface(Node):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, subClass=False, **kwArgs):
		""" Create a Viewer ROS node. """
		newKw = { "node_name": "rt_bi_core_sensor", **kwArgs}
		super().__init__(**newKw)
		if subClass:
			self.get_logger().debug("%s in sensor topic init." % self.get_fully_qualified_name())
		else:
			self.get_logger().debug("%s is initializing." % self.get_fully_qualified_name())
			RosUtils.SetLogger(self.get_logger())
		self.__sensors: Dict[int, SensorRegion] = {}
		if subClass:
			self.get_logger().debug("%s skipping creating publishers and subscribers." % self.get_fully_qualified_name())
		else:
			SaMsgs.subscribeToRobotStateTopic(self, partial(self.__onRobotStateUpdate, render=True))
			SaMsgs.subscribeToEstimationTopic(self, self.__onEstimation)
			(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))

	def __onRobotStateUpdate(self, update: RobotState, render = False) -> Union[SensorRegion, None]:
		if update is None:
			self.get_logger().warn("Received empty RobotState!")
			return

		self.get_logger().debug("Updating sensor %d definition." % update.robot_id)
		coords = SaMsgs.convertSaPoseListToCoordsList(update.fov.corners)
		timeNanoSecs = self.get_clock().now().nanoseconds
		tracklets = self.__sensors[update.robot_id].tracklets if update.robot_id in self.__sensors else {}
		sensor = SensorRegion(centerOfRotation=SaMsgs.convertSaPoseToPose(update.pose), idNum=update.robot_id, envelope=coords, timeNanoSecs=timeNanoSecs, tracklets=tracklets)
		self.__sensors[update.robot_id] = sensor
		if render: self.render([sensor])
		return sensor

	def __onEstimation(self, msg: EstimationMsg) -> None:
		if msg is None:
			self.get_logger().warn("Received empty Estimation!")
			return

		toRender = []
		for poseEstimation in msg.pose_estimations:
			self.get_logger().info("Received TR#%d" % poseEstimation.trajectory_id)
			pose = SaMsgs.convertSaPoseToPose(poseEstimation.pose)
			tracklet = Tracklet(poseEstimation.trajectory_id, int(msg.detection_time), pose.x, pose.y, pose.angleFromX)
			if msg.robot_state.robot_id in self.__sensors:
				self.__sensors[msg.robot_state.robot_id].addTracklet(tracklet)
				toRender.append(msg.robot_state.robot_id)
		self.render([self.__sensors[i] for i in toRender])
		return

	def render(self, regions: Union[List[SensorRegion], List[Tracklet]]) -> None:
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
	node = SensorTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
