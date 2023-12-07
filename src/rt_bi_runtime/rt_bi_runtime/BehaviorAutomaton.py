import json
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion


class BehaviorAutomaton(Node):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self) -> None:
		""" Create a Behavior Automaton node. """
		super().__init__(node_name="rt_bi_core_bi") # type: ignore - parameter_overrides: List[Parameter] = None
		self.get_logger().info("%s is initializing." % self.get_fully_qualified_name())
		self.__declareParameters()
		self.__name: str = self.get_fully_qualified_name()
		self.__states: List[str] = []
		self.__transitions: List[List[tuple]] = []
		self.__start: str = self.get_fully_qualified_name()
		self.__accepting: List[str] = []
		self.__parseConfigFileParameters()
		RosUtils.SetLogger(self.get_logger())
		return

	def __repr__(self) -> str:
		return self.__name

	def __declareParameters(self) -> None:
		self.get_logger().debug("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("name", Parameter.Type.STRING)
		self.declare_parameter("states", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions", Parameter.Type.STRING_ARRAY)
		return

	def __parseConfigFileParameters(self) -> None:
		self.get_logger().debug("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__name = self.get_parameter("name").get_parameter_value().string_value
		self.__states = list(self.get_parameter("states").get_parameter_value().string_array_value)
		transitionsLists = list(self.get_parameter("transitions").get_parameter_value().string_array_value)
		self.__transitions = [json.loads(transitionsList) for transitionsList in transitionsLists]
		self.__start = self.get_parameter("start").get_parameter_value().string_value
		self.__accepting = list(self.get_parameter("transitions").get_parameter_value().string_array_value)
		# matches = RegEx.search(r"\((.*),\s+(.*)\)", self.specifier)
		# if matches is None: raise ValueError("Unexpected specifier format: %s" % specifier)
		# self.symbol = matches.group(1)
		# self.consuming = json.loads(matches.group(2).lower())
		# self.validator: Symbol = validators[self.name]
		return

	def render(self, regions: List[PolygonalRegion]) -> None:
		self.get_logger().info("Render BA.")
		return

def main(args=None):
	rclpy.init(args=args)
	node = BehaviorAutomaton()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
