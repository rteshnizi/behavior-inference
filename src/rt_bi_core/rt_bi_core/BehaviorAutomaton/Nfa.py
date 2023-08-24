import json
import re as RegEx
from typing import Dict, List, Literal, Set, Union

import matplotlib.pyplot as plt
import networkx as nx

from rt_bi_core.BehaviorAutomaton.Symbol import Symbol
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.ShadowTree.ConnectivityGraph import ConnectivityGraph
from rt_bi_core.ShadowTree.Graph import GraphAlgorithms


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

	def getShapelyPolygon(self, cGraph: ConnectivityGraph):
		cluster = cGraph.nodeToClusterMap[self.path]
		return cGraph.nodeClusters[cluster].polygon

class Transition:
	def __init__(self, specifier, validators):
		self.specifier = specifier
		matches = RegEx.search(r"\((.*),\s+(.*)\)", self.specifier)
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
		path = GraphAlgorithms.bfs(shadowTree, penny.path[-1], self.validator.lambdaObj.func)
		return None if path is None else path

SpecName = Literal["TET1", "TWIST"]

class Nfa(nx.DiGraph):
	def __init__(self, specName: SpecName, states, transitions, validators):
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
		self._previousCGraph: ConnectivityGraph = None

	def __repr__(self):
		return "%s.NFA" % self._specName

	def __buildGraph(self):
		if self.START_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.START_SYMBOL)
		if self.TERMINAL_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.TERMINAL_SYMBOL)
		self.add_nodes_from(self.states)
		for fromState in self.transitions:
			for i in range(len(self.transitions[fromState])):
				matches = RegEx.search(r"\((\(.*\)),\s+(.*)\)", self.transitions[fromState][i])
				transition = Transition(matches.group(1), self.validators)
				toState = matches.group(2)
				self.add_edge(fromState, toState, transition=transition)

	def displayGraph(self):
		fig = plt.figure(len(GraphAlgorithms.__allFigs))
		GraphAlgorithms.__allFigs.add(fig)
		pos = nx.circular_layout(self)
		activeStates = { penny.state for penny in self.activeStates }
		inactiveStates = set(self.nodes) - activeStates
		nx.draw_networkx_nodes(self, pos, nodelist=activeStates, node_color="PaleGreen")
		nx.draw_networkx_nodes(self, pos, nodelist=inactiveStates - {"END"}, node_color="lightgrey")
		# END STATE
		nx.draw_networkx_nodes(self, pos, nodelist={"END"}, node_color="tomato" if "END" in inactiveStates else "PaleGreen")
		nx.draw_networkx_edges(self, pos)
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		edgeLabel = nx.get_edge_attributes(self, "transition")
		nx.draw_networkx_edge_labels(self, pos, edge_labels=edgeLabel)
		pennyText = [repr(penny) for penny in self.activeStates]
		plt.annotate("\n".join(pennyText), xy=(0.01, 0), xycoords="axes fraction") # cSpell:ignore xycoords
		plt.axis("off")
		fig.show()
		self._fig = fig

	def killDisplayedGraph(self):
		if self._fig is not None:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
