from typing import Any

import rclpy
from rclpy.logging import LoggingSeverity

from rt_bi_core.RegionsSubscriber import TargetSubscriber


class TargetRenderer(TargetSubscriber):
	""" This Node listens to all the messages published on the topics related to targets and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_target", **kwArgs}
		super().__init__(loggingSeverity=LoggingSeverity.INFO, **newKw)

	def onRegionsUpdated(self, __1: Any, __2: Any) -> None:
		self.log("Targets updated.")
		self.render()
		return

	def declareParameters(self) -> None:
		return

	def parseParameters(self) -> None:
		return

def main(args=None):
	rclpy.init(args=args)
	node = TargetRenderer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
