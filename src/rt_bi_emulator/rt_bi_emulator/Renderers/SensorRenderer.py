from typing import Any

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import SensorSubscriber


class SensorRenderer(SensorSubscriber):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_sensor", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)

	def onSensorUpdated(self, polygon: Any) -> None:
		self.log("Sensors updated.")
		self.render()
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		for regionId in self.sensorRegions:
			polys = self.sensorRegions[regionId]
			for poly in polys:
				marker = poly.createMarkers(durationNs=-1, stamped=False, renderTracklet=True)
				Ros.ConcatMessageArray(markers, marker)
		return markers

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
