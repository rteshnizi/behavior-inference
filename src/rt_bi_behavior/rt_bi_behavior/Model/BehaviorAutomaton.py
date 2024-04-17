from json import dumps
from tempfile import TemporaryFile

import networkx as nx
from networkx.drawing import nx_agraph

from rt_bi_behavior.Model.State import State, StateToken
from rt_bi_behavior.Model.Transition import Transition
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class BehaviorAutomaton(nx.DiGraph):
	def __init__(self,
			specName: str,
			states: list[str],
			transitions: dict[str, dict[str, str]],
			start: str,
			accepting: list[str],
			baseDir: str,
			transitionGrammarDir: str,
			grammarFileName: str,
		):
		super().__init__()
		self.__dotPublisher: Ros.Publisher | None = None
		self.__specName: str = specName
		self.name = f"NFA({self.__specName})"
		self.__states: list[str] = states
		self.__transitions: dict[str, dict[str, str]] = transitions
		self.__start: str = start
		self.__accepting: set[str] = set(accepting)
		self.__predicates: set[str] = set()
		self.__baseDir: str = baseDir
		self.__transitionGrammarDir: str = transitionGrammarDir
		self.__grammarFileName: str = grammarFileName
		self.__tokens: set[StateToken] = set()
		self.__buildGraph()
		return

	def __repr__(self):
		return self.name

	def __addNode(self, name: str) -> None:
		state = State(name, starting=(name == self.__start), accepting=(name in self.__accepting))
		self.add_node(name, descriptor=state)
		return

	def __addEdge(self, source: str, transitionStr: str, destination: str) -> None:
		transition = Transition(transitionStr, self.__baseDir, self.__transitionGrammarDir, self.__grammarFileName)
		self.__predicates |= transition.predicates
		self.add_edge(source, destination, label=str(transition), descriptor=transition)
		return

	def __buildGraph(self) -> None:
		for n in self.__states:
			self.__addNode(n)
		for i in range(len(self.__states)):
			src = self.__states[i]
			if src not in self.__transitions: continue
			for dst in self.__transitions[src]:
				filterStr = self.__transitions[src][dst]
				self.__addEdge(src, filterStr, dst)
		return

	@property
	def predicates(self) -> list[str]:
		return list(self.__predicates)

	@property
	def tokens(self) -> set[StateToken]:
		return self.__tokens

	def resetTokens(self, topologicalGraphNode: list[NodeId]) -> None:
		for n in topologicalGraphNode:
			self.__tokens.add(StateToken(
				stateName=self.__start,
				graphNode=n,
			))
		return

	def initFlask(self, rosNode: Ros.Node) -> None:
		(self.__dotPublisher, _) = Ros.CreatePublisher(
			rosNode,
			Msgs.Std.String,
			"/rt_bi_behavior/dot_renderer",
			callbackFunc=self.render,
			intervalSecs=1,
		)
		return

	def render(self) -> None:
		if self.__dotPublisher is None: return
		# self.get_logger().error("HERE IN BA RENDER")
		with TemporaryFile() as f:
			nx_agraph.to_agraph(self).draw(path=f, prog="dot", format="svg")
			f.seek(0)
			svg = f.read().decode()
			self.__dotPublisher.publish(Msgs.Std.String(data=dumps({"name": self.name, "svg": svg})))
		return
