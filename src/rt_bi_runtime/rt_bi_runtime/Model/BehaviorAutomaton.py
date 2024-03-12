from typing import Generator

import networkx as nx

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_runtime.Model.State import State
from rt_bi_runtime.Model.Transition import Transition


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
		self.__specName: str = specName
		self.name = "NFA(%s)" % self.__specName
		self.__states: list[str] = states
		self.__transitions: dict[str, dict[str, str]] = transitions
		self.__start: str = start
		self.__accepting: set[str] = set(accepting)
		self.__predicates: set[str] = set()
		self.__baseDir: str = baseDir
		self.__transitionGrammarDir: str = transitionGrammarDir
		self.__grammarFileName: str = grammarFileName
		self.__buildGraph()
		self.renderLayout = nx.circular_layout(self)
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
		self.add_edge(source, destination, descriptor=transition)
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
	def acceptingStates(self) -> set[str]:
		return self.__accepting

	def activeStates(self) -> set[str]:
		Ros.Logger().warn(f"Not implemented.")
		return set()

	def nonActiveStates(self) -> set[str]:
		return (set(self.__states) - self.activeStates())

	def baGenerator(self) -> Generator["BehaviorAutomaton", None, None]:
		yield self
