import re as RegEx
from typing import Dict, Set, Union

import networkx as nx

from rt_bi_runtime.Model.Symbol import Symbol
from rt_bi_runtime.rt_bi_runtime.Model.StateSummary import StateSummary
from rt_bi_runtime.rt_bi_runtime.Model.Transition import Transition


class Nfa(nx.DiGraph):
	def __init__(self, specName: str, states, transitions, validators):
		super().__init__()
		self._specName = specName
		self.states = states
		self.transitions = transitions
		self.validators: Dict[str, Symbol] = validators
		self.__START_SYMBOL = "START"
		self.__TERMINAL_SYMBOL = "END"
		self._fig = None
		self.activeStates: Set[StateSummary] = set()
		self.__buildGraph()
		return

	def __repr__(self):
		return "%s.NFA" % self._specName

	def __buildGraph(self):
		if self.__START_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.__START_SYMBOL)
		if self.__TERMINAL_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.__TERMINAL_SYMBOL)
		self.add_nodes_from(self.states)
		for fromState in self.transitions:
			for i in range(len(self.transitions[fromState])):
				matches = RegEx.search(r"\((\(.*\)),\s+(.*)\)", self.transitions[fromState][i])
				if matches is None: raise ValueError("Unexpected transition format: %s" % self.transitions[fromState][i])
				# transition = Transition(matches.group(1), self.validators)
				# toState = matches.group(2)
				# self.add_edge(fromState, toState, transition=transition)
		return

	def displayGraph(self) -> None:
		return

	def killDisplayedGraph(self) -> None:
		return
