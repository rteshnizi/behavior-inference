import rclpy
from rclpy.node import Node

import rt_bi_utils.Ros as RosUtils


class SpGraphInterface(Node):
	def __init__(self) -> None:
		super().__init__(node_name="rt_bi_core_sp_graph") # type: ignore - parameter_overrides: List[Parameter] = None
		self.get_logger().info("%s is initializing." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		return


def main(args=None):
	rclpy.init(args=args)
	ba = SpGraphInterface()
	rclpy.spin(ba)
	ba.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
