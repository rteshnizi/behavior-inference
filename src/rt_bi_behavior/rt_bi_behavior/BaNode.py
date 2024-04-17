from json import dumps
from tempfile import TemporaryFile

import rclpy
from ament_index_python.packages import get_package_share_directory
from networkx.drawing import nx_agraph
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from rt_bi_behavior import package_name
from rt_bi_behavior.Model.BehaviorAutomaton import BehaviorAutomaton
from rt_bi_behavior.Model.TopologicalGraph import TopologicalGraph
from rt_bi_commons.Base.ColdStartableNode import ColdStartableNode, ColdStartPayload
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces


class BaNode(ColdStartableNode):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self, **kwArgs) -> None:
		""" Create a Behavior Automaton node. """
		newKw = { "node_name": "ba", "loggingSeverity": LoggingSeverity.WARN, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.__name: str = self.get_fully_qualified_name()
		self.__baseDir = get_package_share_directory(package_name)
		self.__dotPublisher: Ros.Publisher | None = None
		self.__grammarDir: str = ""
		self.__grammarFile: str = ""
		self.__states: list[str] = []
		self.__transitions: dict[str, dict[str, str]] = {}
		self.__start: str = ""
		self.__accepting: list[str] = []
		self.parseParameters()
		self.__ba = BehaviorAutomaton(
			self.__name,
			self.__states,
			self.__transitions,
			self.__start,
			self.__accepting,
			self.__baseDir,
			self.__grammarDir,
			self.__grammarFile
		)
		self.__rdfClient = RtBiInterfaces.createSpaceTimeClient(self)
		Ros.WaitForServiceToStart(self, self.__rdfClient)
		self.waitForColdStartPermission(self.onColdStartAllowed)
		RtBiInterfaces.subscribeToEventGraph(self, self.__onInitGraph)
		RtBiInterfaces.subscribeToEvent(self, self.__onEvent)
		return

	def __onInitGraph(self, msg: Msgs.RtBi.Graph) -> None:
		assert isinstance(msg.vertices, list)
		assert isinstance(msg.adjacency, list)
		assert len(msg.vertices) == len(msg.adjacency), f"Vertices and adjacency must have equal length: V:{len(msg.vertices)}, A:{len(msg.adjacency)}"
		self.__topologicalGraph = TopologicalGraph(msg.vertices, msg.adjacency, rVizPublisher=None)
		ids: list[NodeId] = [n for n in self.__topologicalGraph.nodes]
		self.__ba.resetTokens(ids)
		Ros.Log(f"Topological Graph: {len(self.__topologicalGraph.nodes)} vertices and {len(self.__topologicalGraph.edges)} edges")
		return

	def __onEvent(self, msg: Msgs.RtBi.Events) -> None:
		assert isinstance(msg.component_events, list)
		Ros.Log(f"Topological Events: {len(msg.component_events)}.")
		# Ros.Log(f"Before applying: V={len(self.__topologicalGraph.nodes)}, E={len(self.__topologicalGraph.edges)}.")
		# for event in msg.component_events: self.__topologicalGraph.apply(event)
		# Ros.Log(f"After applying: V={len(self.__topologicalGraph.nodes)}, E={len(self.__topologicalGraph.edges)}.")
		# Ros.Log("Tokens", self.__ba.tokens)
		# self.__timer = Ros.CreateTimer(self, self.__evaluateTokens, 500)
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		if self.shouldRender: (self.__dotPublisher, _) = Ros.CreatePublisher(
			self,
			Msgs.Std.String,
			"/rt_bi_behavior/dot_renderer",
			callbackFunc=self.render,
			intervalSecs=1,
		)
		super().coldStartCompleted({
			"done": True,
			"phase": payload.phase,
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

	def render(self, msg=None) -> None:
		if not self.shouldRender: return
		if self.__dotPublisher is None: return
		# self.get_logger().error("HERE IN BA RENDER")
		with TemporaryFile() as f:
			nx_agraph.to_agraph(self.__ba).draw(path=f, prog="dot", format="svg")
			f.seek(0)
			svg = f.read().decode()
			# self.get_logger().error(f"HERE IN BA RENDER: {svg}")
			self.__dotPublisher.publish(Msgs.Std.String(data=dumps({"name": self.__ba.name, "svg": svg})))
		# FIXME: Set the params for dot file here
# 		if self.__dotRenderer is None:
# 			return
# 		Ros.Wait(self, 2)
# 		dotStr = \
# """digraph finite_state_machine {
# 	fontname="Helvetica,Arial,sans-serif"
# 	node [fontname="Helvetica,Arial,sans-serif"]
# 	edge [fontname="Helvetica,Arial,sans-serif"]
# 	rankdir=LR;
# 	node [shape = doublecircle]; 0 3 4 8;
# 	node [shape = circle];
# 	0 -> 2 [label = "SS(B)"];
# 	0 -> 1 [label = "SS(S)"];
# 	1 -> 3 [label = "S($end)"];
# 	2 -> 6 [label = "SS(b)"];
# 	2 -> 5 [label = "SS(a)"];
# 	2 -> 4 [label = "S(A)"];
# 	5 -> 7 [label = "S(b)"];
# 	5 -> 5 [label = "S(a)"];
# 	6 -> 6 [label = "S(b)"];
# 	6 -> 5 [label = "S(a)"];
# 	7 -> 8 [label = "S(b)"];
# 	7 -> 5 [label = "S(a)"];
# 	8 -> 6 [label = "S(b)"];
# 	8 -> 5 [label = "S(a)"];
# }"""
# 		self.get_logger().error("RENDER")
# 		self.__dotRenderer.updateSVGStr(dotStr)
# 		return

def main(args=None):
	rclpy.init(args=args)
	ba = BaNode()
	rclpy.spin(ba)
	ba.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
