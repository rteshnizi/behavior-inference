from typing import Any, cast

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import SensorSubscriber
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet


class SensorRenderer(SensorSubscriber):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_sensor", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(pauseQueuingMsgs=False, **newKw)
		RtBiInterfaces.subscribeToEstimation(self, self.__onEstimation)

	def onPolygonUpdated(self, rType: Any, msgs: Any) -> None:
		self.log("Sensors updated.")
		self.render()
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for regionId in self.sensorRegions:
			polys = self.sensorRegions[regionId]
			for poly in polys:
				marker = poly.createMarkers(durationNs=-1, stamped=False)
				Ros.ConcatMessageArray(markers, marker)
		return markers

	def __onEstimation(self, msg: Msgs.RtBi.Estimation) -> None:
		if msg is None:
			self.get_logger().warn("Received empty Estimation!")
			return

		for trackletMsg in msg.estimations:
			trackletMsg = cast(Msgs.RtBi.Tracklet, trackletMsg)
			pose = cast(Msgs.Geometry.Pose, trackletMsg.pose)
			tracklet = Tracklet(
				idStr=trackletMsg.id,
				timeNanoSecs=Msgs.toNanoSecs(msg.robot.stamp),
				hIndex=-1,
				x=pose.position.x,
				y=pose.position.y,
				angleFromX=Msgs.toAngle(pose.orientation),
				spawned=tracklet.spawned,
				vanished=tracklet.vanished,
			)
			id = SensingPolygon.Id(
				hIndex=-1,
				timeNanoSecs=Msgs.toNanoSecs(msg.robot.stamp),
				regionId=msg.robot.id,
				polygonId=Ros.GetMessage(msg.robot.polygons, 0, Msgs.RtBi.Polygon).id,
			)
			(regionId, index) = self.sensorPolyIdMap[id]
			self.sensorRegions[regionId][index].tracklets[tracklet.id] = tracklet
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
