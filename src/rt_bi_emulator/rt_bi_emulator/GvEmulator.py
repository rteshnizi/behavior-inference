import rclpy
from rclpy.node import Node

import rt_bi_utils.Ros as RosUtils


class GvEmulator(Node):
	""" The Viewer ROS Node """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_gv_emulator")
		self.get_logger().info("GV Emulator is starting...")
		RosUtils.SetLogger(self.get_logger())

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	gvNode = GvEmulator()
	rclpy.spin(gvNode)
	gvNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()