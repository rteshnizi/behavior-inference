from typing import cast

import rclpy
from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

from rt_bi_commons.Base.RegionsSubscriber import RegionsSubscriberBase, SensorSubscriber
from rt_bi_commons.Shared.Tracklet import Tracklet
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Polygons.SensingPolygon import SensingPolygon


class SensorRenderer(SensorSubscriber):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_sensor", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		RtBiInterfaces.subscribeToEstimation(self, self.__onEstimation)

	def onSensorsUpdated(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} sensors updated.")
		self.render()
		return

	def __onEstimation(self, msg: Msgs.RtBi.Estimation) -> None:
		if msg is None:
			self.get_logger().warn("Received empty Estimation!")
			return

		for trackletMsg in msg.estimations:
			trackletMsg = cast(Msgs.RtBi.Tracklet, trackletMsg)
			pose = cast(Msgs.Geometry.Pose, trackletMsg.pose)
			tracklet = Tracklet(
				trackletMsg.id,
				Msgs.toNanoSecs(msg.robot.stamp),
				pose.position.x,
				pose.position.y,
				Msgs.toAngle(pose.orientation),
				tracklet.spawned,
				tracklet.vanished
			)
			id_ = SensingPolygon.Id(regionId=msg.robot.id, polygonId=Ros.GetMessage(msg.robot.polygons, 0, Msgs.RtBi.Polygon).id)
			self.sensors[id_.regionId][id_].tracklets[tracklet.id] = tracklet
		self.render()
		return

	def declareParameters(self) -> None:
		return super().declareParameters()

	def parseParameters(self) -> None:
		return super().parseParameters()

def main(args=None):
	rclpy.init(args=args)
	node = SensorRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
