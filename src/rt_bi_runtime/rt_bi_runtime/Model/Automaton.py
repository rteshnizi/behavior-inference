from typing import List

import rclpy
from rclpy.parameter import Parameter

from rt_bi_commons.Base.RtBiNode import RtBiNode


class DataDictionary(RtBiNode):
	def __init__(self):
		super().__init__(node_name="em_dd")
		self.declareParameters()
		self.__alphabet: List[str] = []
		self.__types: List[str] = []
		self.__descriptions: List[str] = []
		self.parseParameters()

	def declareParameters(self) -> None:
		self.log("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("alphabet", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("types", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("descriptions", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__alphabet = list(self.get_parameter("alphabet").get_parameter_value().string_array_value)
		self.__types = list(self.get_parameter("types").get_parameter_value().string_array_value)
		self.__descriptions = list(self.get_parameter("descriptions").get_parameter_value().string_array_value)
		return

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	ddNode = DataDictionary()
	rclpy.spin(ddNode)
	ddNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
