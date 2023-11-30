import json
import re as RegEx
from typing import Dict, List, Set, Union

import networkx as nx

from rt_bi_core.Eventifier.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.Eventifier.ShadowTree import ShadowTree
from rt_bi_runtime.BehaviorInference.Symbol import Symbol


class Penny:
	"""
		This is the weirdest name I could use.
		This represents a penny in what Dr. Shell referred to as a stack of pennies that will track a possible pose and its state.
		A penny represents an active hypothesis about the placement of a hidden target.
	"""
	def __init__(self, state: str, path: List[str]):
		"""
		Create a penny.

		Parameters
		----------
		state : str
			The name of the state in the NFA graph.
		path : List[str]
			The path in the shadow graph that has so far carried this penny over the transitions.
		"""
		self.state = state
		self.path = path

	def __repr__(self):
		return "(%s, %s)" % (self.state, self.path)

	def __hash__(self):
		return hash(repr(self))

class Transition:
	def __init__(self, specifier: str, validators):
		self.specifier = specifier
		matches = RegEx.search(r"\((.*),\s+(.*)\)", self.specifier)
		if matches is None: raise ValueError("Unexpected specifier format: %s" % specifier)
		self.name = matches.group(1)
		self.consuming = json.loads(matches.group(2).lower())
		self.validator: Symbol = validators[self.name]

	def __repr__(self):
		return repr(self.specifier)

	def execute(self, penny: Penny, shadowTree: ShadowTree) -> Union[List[str], None]:
		"""
			Here we check whether any node in the shadow tree satisfy the validator for this node
			#### Returns
				The path that satisfied the validator function or `None`
		"""
		return

class Nfa(nx.DiGraph):
	def __init__(self, specName: str, states, transitions, validators):
		super().__init__()
		self._specName = specName
		self.states = states
		self.transitions = transitions
		self.validators: Dict[str, Symbol] = validators
		self.START_SYMBOL = "START"
		self.TERMINAL_SYMBOL = "END"
		self._fig = None
		self.activeStates: Set[Penny] = set()
		self.__buildGraph()
		self._previousCGraph: Union[ConnectivityGraph, None] = None

	def __repr__(self):
		return "%s.NFA" % self._specName

	def __buildGraph(self):
		if self.START_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.START_SYMBOL)
		if self.TERMINAL_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.TERMINAL_SYMBOL)
		self.add_nodes_from(self.states)
		for fromState in self.transitions:
			for i in range(len(self.transitions[fromState])):
				matches = RegEx.search(r"\((\(.*\)),\s+(.*)\)", self.transitions[fromState][i])
				if matches is None: raise ValueError("Unexpected transition format: %s" % self.transitions[fromState][i])
				transition = Transition(matches.group(1), self.validators)
				toState = matches.group(2)
				self.add_edge(fromState, toState, transition=transition)

	def displayGraph(self) -> None:
		return

	def killDisplayedGraph(self) -> None:
		return
