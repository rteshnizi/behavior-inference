from typing import Generator

import networkx as nx

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_interfaces.srv import SpaceTime


class Transitions:
	@staticmethod
	def SouthBank() -> SpaceTime.Request:
		req = SpaceTime.Request()
		req.filter.name = "south-bank"
		return req

	@staticmethod
	def MidDay() -> SpaceTime.Request:
		req = SpaceTime.Request()
		req.filter.name = "mid-day"
		return req

	@staticmethod
	def NorthShore() -> SpaceTime.Request:
		req = SpaceTime.Request()
		req.filter.name = "north-shore"
		return req

class BehaviorAutomaton(nx.DiGraph):
	class State:
		def __init__(self, name: str, starting: bool, accepting: bool) -> None:
			self.name = name
			self.start = starting
			self.accepting = accepting

		def __repr__(self) -> str:
			return "(%s-%s-%s)" % (self.name, "T" if self.start else "F", "T" if self.accepting else "F")

		def __hash__(self) -> int:
			return hash(repr(self))

		def __eq__(self, other: "BehaviorAutomaton.State") -> bool:
			return self.name == other.name

	class Transition:
		def __init__(self, symbolName: str) -> None:
			self.symbolName: str = symbolName

		def __repr__(self) -> str:
			return "%s" % (self.symbolName)

		def __hash__(self) -> int:
			return hash(self.symbolName)

		def __eq__(self, other: "BehaviorAutomaton.Transition") -> bool:
			return self.symbolName == other.symbolName

		def spaceTimeRequest(self) -> SpaceTime.Request:
			try:
				req: SpaceTime.Request = eval(self.symbolName)
				return req
			except Exception as e:
				Ros.Logger().error(f"Failed to evaluate transition function: {repr(e)}")
				return SpaceTime.Request()

	def __init__(self, specName: str, states: list[str], transitions: list[tuple[str, str]], start: str, accepting: list[str]):
		super().__init__()
		self.__specName: str = specName
		self.name = "NFA(%s)" % self.__specName
		self.__states: list[str] = states
		self.__transitions: list[tuple[str, str]] = transitions
		self.__start: str = start
		self.__accepting: set[str] = set(accepting)
		self.__buildGraph()
		self.renderLayout: NxUtils.GraphLayout | NxUtils.NxDefaultLayout = nx.circular_layout(self)
		return

	def __repr__(self):
		return self.name

	def __addNode(self, name: str) -> None:
		state = BehaviorAutomaton.State(name, starting=(name == self.__start), accepting=(name in self.__accepting))
		self.add_node(name, descriptor=state)
		return

	def __addEdge(self, source: str, symbolName: str, destination: str) -> None:
		transition = BehaviorAutomaton.Transition(symbolName)
		self.add_edge(source, destination, descriptor=transition)
		return

	def __buildGraph(self) -> None:
		if len(self.__states) != len(self.__transitions):
			raise AssertionError("The length of states and transitions arrays must be equal. S=%d, T=%d" % (len(self.__states), len(self.__transitions)))

		for n in self.__states:
			self.__addNode(n)
		for i in range(len(self.__states)):
			src = self.__states[i]
			for transition in self.__transitions[i]:
				(sym, dst) = (transition[0], transition[1])
				self.__addEdge(src, sym, dst)
		return

	@property
	def acceptingStates(self) -> set[str]:
		return self.__accepting

	def activeStates(self) -> set[str]:
		return set()

	def nonActiveStates(self) -> set[str]:
		return (set(self.__states) - self.activeStates())

	def baGenerator(self) -> Generator["BehaviorAutomaton", None, None]:
		yield self

	def allSymbols(self) -> list[SpaceTime.Request]:
		transitionsDict: dict[tuple[str, str], BehaviorAutomaton.Transition] = nx.get_edge_attributes(self, "descriptor")
		transitions = transitionsDict.values()
		return [t.spaceTimeRequest() for t in transitions]

	def spatialSymbols(self) -> list[str]: ...
	def temporalSymbols(self) -> list[str]: ...
