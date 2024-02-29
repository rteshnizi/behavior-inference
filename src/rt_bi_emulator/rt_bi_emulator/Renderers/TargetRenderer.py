import rclpy
from rclpy.logging import LoggingSeverity
from visualization_msgs.msg import MarkerArray

from rt_bi_commons.Base.RegionsSubscriber import TargetSubscriber
from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import ColorNames, RViz
from rt_bi_core.Polygons.TargetPolygon import TargetPolygon
from rt_bi_interfaces.msg import RegularSpace


class TargetRenderer(TargetSubscriber):
	""" This Node listens to all the messages published on the topics related to targets and renders them. """
	def __init__(self, **kwArgs):
		newKw = { "node_name": "renderer_target", **kwArgs}
		super().__init__(loggingSeverity=LoggingSeverity.INFO, **newKw)

	def onTargetsUpdated(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} targets updated.")
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
