from typing import Any

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_core.RegionsSubscriber import MapSubscriber


class MapRenderer(MapSubscriber):
	""" This Node listens to all static and dynamic map region updates and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_map", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)

	def onRegionsUpdated(self, __1: Any, __2: Any) -> None:
		self.log("Map updated.")
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	node = MapRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
