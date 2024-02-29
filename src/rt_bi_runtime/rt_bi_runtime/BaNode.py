import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_runtime import package_name
from rt_bi_runtime.Model.BaMatplotlibRenderer import BaMatplotlibRenderer
from rt_bi_runtime.Model.BehaviorAutomaton import BehaviorAutomaton


class BaNode(ColdStartableNode):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self, **kwArgs) -> None:
		""" Create a Behavior Automaton node. """
		newKw = { "node_name": "ba", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.__name: str = self.get_fully_qualified_name()
		self.__grammarDir: str = ""
		self.__grammarFile: str = ""
		self.__states: list[str] = []
		self.__transitions: dict[str, dict[str, str]] = {}
		self.__start: str = ""
		self.__accepting: list[str] = []
		self.parseParameters()
		self.rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServicesToStart(self, self.rdfClient)
		baseDir = get_package_share_directory(package_name)
		self.__ba = BehaviorAutomaton(
			self.__name,
			self.__states,
			self.__transitions,
			self.__start,
			self.__accepting,
			baseDir,
			self.__grammarDir,
			self.__grammarFile
		)
		self.waitForColdStartPermission(self.onColdStartAllowed)
		# if self.shouldRender: self.render()
		return

	def onColdStartAllowed(self, _) -> None:
		super().coldStartCompleted({
			"done": True,
			"predicates": self.__ba.predicates,
		})
		return

	def declareParameters(self) -> None:
		self.log("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("grammar_dir", Parameter.Type.STRING)
		self.declare_parameter("grammar_file", Parameter.Type.STRING)
		self.declare_parameter("states", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_from", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_predicate", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_to", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("start", Parameter.Type.STRING)
		self.declare_parameter("accepting", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__name = self.get_fully_qualified_name()
		self.__grammarDir = self.get_parameter("grammar_dir").get_parameter_value().string_value
		self.__grammarFile = self.get_parameter("grammar_file").get_parameter_value().string_value
		self.__states = list(self.get_parameter("states").get_parameter_value().string_array_value)
		frmList: list[str] = list(self.get_parameter("transitions_from").get_parameter_value().string_array_value)
		prdList: list[str] = list(self.get_parameter("transitions_predicate").get_parameter_value().string_array_value)
		toList: list[str] = list(self.get_parameter("transitions_to").get_parameter_value().string_array_value)
		for i in range(len(frmList)):
			frmState = frmList[i]
			toState = toList[i]
			prd = prdList[i]
			if frmState not in self.__transitions: self.__transitions[frmState] = {}
			self.__transitions[frmState][toState] = prd

		self.__start = self.get_parameter("start").get_parameter_value().string_value
		self.__accepting = list(self.get_parameter("accepting").get_parameter_value().string_array_value)
		return

	def render(self) -> None:
		# FIXME: MATPLOTLIB blocks ros thread.
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
