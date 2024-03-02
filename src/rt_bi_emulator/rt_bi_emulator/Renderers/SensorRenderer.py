from typing import Any, cast

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.RegionsSubscriber import SensorSubscriber
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet


class SensorRenderer(SensorSubscriber):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_sensor", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)
		RtBiInterfaces.subscribeToEstimation(self, self.__onEstimation)

	def onRegionsUpdated(self, __1: Any, __2: Any) -> None:
		self.log("Sensors updated.")
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
			id = SensingPolygon.Id(regionId=msg.robot.id, polygonId=Ros.GetMessage(msg.robot.polygons, 0, Msgs.RtBi.Polygon).id, overlappingRegionId="", overlappingPolygonId="")
			self.sensors[id.regionId][id].tracklets[tracklet.id] = tracklet
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
