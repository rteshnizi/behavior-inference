import json
from typing import List

import rclpy
from rclpy.parameter import Parameter

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_runtime.Model.BaMatplotlibRenderer import BaMatplotlibRenderer
from rt_bi_runtime.Model.BehaviorAutomaton import BehaviorAutomaton


class BaNode(RtBiNode):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self) -> None:
		""" Create a Behavior Automaton node. """
		super().__init__(node_name="rt_bi_core_bi")
		self.declareParameters()
		self.__name: str = self.get_fully_qualified_name()
		self.__states: List[str] = []
		self.__transitions: List[tuple[str, str]] = []
		self.__start: str = ""
		self.__accepting: List[str] = []
		self.parseParameters()
		self.__ba = BehaviorAutomaton(self.__name, self.__states, self.__transitions, self.__start, self.__accepting)
		self.__coldStart()
		if self.shouldRender: self.render()
		return

	def __coldStart(self) -> None:

		return

	def declareParameters(self) -> None:
		self.log("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("states", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("start", Parameter.Type.STRING)
		self.declare_parameter("accepting", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__name = self.get_fully_qualified_name()
		self.__states = list(self.get_parameter("states").get_parameter_value().string_array_value)
		for transition in list(self.get_parameter("transitions").get_parameter_value().string_array_value):
			self.__transitions.append(json.loads(transition))
		self.__start = self.get_parameter("start").get_parameter_value().string_value
		self.__accepting = list(self.get_parameter("accepting").get_parameter_value().string_array_value)
		return

	def render(self) -> None:
		BaMatplotlibRenderer.createBehaviorAutomatonFigure(self.__ba)
		return

	def destroy_node(self) -> None:
		BaMatplotlibRenderer.closeFig(self.__ba)
		return super().destroy_node()


def main(args=None):
	rclpy.init(args=args)
	ba = BaNode()
	rclpy.spin(ba)
	ba.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
