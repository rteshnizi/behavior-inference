from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import rt_bi_utils.Ros as RosUtils


class DataDictionary(Node):
	""" The Viewer ROS Node """
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__(node_name="rt_bi_emulator_dd") # type: ignore - parameter_overrides: List[Parameter] = None
		self.get_logger().info("%s is initializing." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__declareParameters()
		self.__alphabet: List[str] = []
		self.__types: List[str] = []
		self.__descriptions: List[str] = []
		self.__parseConfigFileParameters()

	def __declareParameters(self) -> None:
		self.get_logger().info("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("alphabet", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("types", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("descriptions", Parameter.Type.STRING_ARRAY)
		return

	def __parseConfigFileParameters(self) -> None:
		self.get_logger().info("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__alphabet = list(self.get_parameter("alphabet").get_parameter_value().string_array_value)
		self.__types = list(self.get_parameter("types").get_parameter_value().string_array_value)
		self.__descriptions = list(self.get_parameter("descriptions").get_parameter_value().string_array_value)
		return

def main(args=None):
	rclpy.init(args=args)
	ddNode = DataDictionary()
	rclpy.spin(ddNode)
	ddNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
