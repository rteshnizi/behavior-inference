from typing import Dict, List

import rclpy
from visualization_msgs.msg import MarkerArray

import rt_bi_commons.Utils.Ros as RosUtils
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_commons.Utils.SaMsgs import SaMsgs
from rt_bi_core.SensorRegion import SensorRegion
from rt_bi_core.Tracklet import Tracklet
from rt_bi_interfaces.msg import DynamicRegion, EstimationMsg


class SensorTopicRenderer(RtBiNode):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_sensor", **kwArgs}
		super().__init__(**newKw)
		self.__sensors: Dict[int, SensorRegion] = {}
		RtBiInterfaces.subscribeToSensor(self, self.__onRegionUpdate)
		RtBiInterfaces.subscribeToEstimation(self, self.__onEstimation)
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self, RosUtils.CreateTopicName("map"))

	def __onRegionUpdate(self, update: DynamicRegion) -> None:
		if update is None:
			self.log("Skipping None update.")
			return

		timeNanoSecs = self.get_clock().now().nanoseconds
		self.log(f"Updating region type {SensorRegion.RegionType.SENSING} id {update.id} definition @{timeNanoSecs}.")
		coords = RtBiInterfaces.fromStdPoints32ToCoordsList(update.region.points)
		cor = RtBiInterfaces.fromStdPointToCoords(update.center_of_rotation)
		tracklets = self.__sensors[update.id].tracklets if update.id in self.__sensors else {}
		sensor = SensorRegion(centerOfRotation=cor, idNum=update.id, envelope=coords, timeNanoSecs=timeNanoSecs, tracklets=tracklets)
		self.__sensors[update.id] = sensor
		self.render([sensor], sensor.tracklets)
		return

	def __onEstimation(self, msg: EstimationMsg) -> None:
		if msg is None:
			self.get_logger().warn("Received empty Estimation!")
			return

		toRender = []
		for poseEstimation in msg.pose_estimations:
			pose = SaMsgs.convertSaPoseToPose(poseEstimation.pose)
			tracklet = Tracklet(poseEstimation.trajectory_id, int(msg.detection_time), pose.x, pose.y, pose.angleFromX, poseEstimation.spawned, poseEstimation.vanished)
			self.__sensors[msg.robot_state.robot_id].tracklets[tracklet.id] = tracklet
			toRender.append(msg.robot_state.robot_id)
		sensors = []
		tracks = {}
		for i in toRender:
			sensors.append(self.__sensors[i])
			tracks = {**tracks, **self.__sensors[i].tracklets}
		self.render(sensors, tracks)
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseParameters(self) -> None:
		return super().parseParameters()

	def render(self, regions: List[SensorRegion], tracklets: Dict[int, Tracklet]) -> None:
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().info("Skipping map render... RViz is not ready yet to receive messages.")
			return
		msg = MarkerArray()
		for region in regions: RosUtils.ConcatMessageArray(msg.markers, region.render())
		for i in tracklets: RosUtils.ConcatMessageArray(msg.markers, tracklets[i].render(SensorRegion.DEFAULT_RENDER_DURATION))
		self.__rvizPublisher.publish(msg)
		return

def main(args=None):
	rclpy.init(args=args)
	node = SensorTopicRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
