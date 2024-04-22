from json import dumps, loads
from tempfile import TemporaryFile
from typing import Any, cast

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
		self.__predicates: dict[str, str] = {}
		"""Predicate to Symbol name map."""
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
		style = "filled" if state.start else ""
		shape = "doublecircle" if state.accepting else "circle"
		self.add_node(name, label=state, shape=shape, style=style)
		return

	def __addEdge(self, source: str, transitionStr: str, destination: str) -> None:
		transition = Transition(transitionStr, self.__baseDir, self.__transitionGrammarDir, self.__grammarFileName)
		for p in transition.predicates: self.__predicates[p] = ""
		self.add_edge(source, destination, label=repr(transition), transition=transition)
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
		return list(self.__predicates.keys())

	def setSymbolicNameOfPredicate(self, symMap: dict[str, str]) -> None:
		"""
		:param symMap: Dictionary from symbolic name to predicate string
		:type symMap: `dict[str, str]`
		"""
		for p in symMap:
			if p in self.__predicates:
				self.__predicates[p] = symMap[p]
		for (frm, to) in self.edges:
			for predicate in self.__predicates:
				symbol = self.__predicates[predicate]
				transition = cast(Transition, self[frm][to]["transition"])
				transition.setPredicatesSymbol(predicate, symbol)
				self[frm][to]["label"] = repr(transition)
		return

	@property
	def tokens(self) -> set[StateToken]:
		return self.__tokens

	def resetTokens(self, shadowNodesStr: str) -> None:
		shadowNodes: list[str] = loads(shadowNodesStr)
		self.__tokens = set()
		for nodeIdJson in shadowNodes:
			token = StateToken.fromNodeIdJson(stateName=self.__start, nodeIdJson=nodeIdJson)
			self.__tokens.add(token)
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

	def __prepareDot(self) -> str:
		with TemporaryFile() as f:
			aGraph = nx_agraph.to_agraph(self)
			aGraph.graph_attr["fontname"] = "Courier"
			aGraph.draw(path=f, prog="dot", format="svg")
			f.seek(0)
			# Ros.Logger().error(str(aGraph))
			svg = f.read().decode()
			return dumps({"name": self.name, "svg": svg, "tokens": [t.asDict() for t in self.tokens]})

	def render(self) -> None:
		if self.__dotPublisher is None: return
		dataStr = self.__prepareDot()
		self.__dotPublisher.publish(Msgs.Std.String(data=dataStr))
		return
