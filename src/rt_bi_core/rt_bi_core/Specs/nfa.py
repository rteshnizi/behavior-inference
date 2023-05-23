import networkx as nx
import matplotlib.pyplot as plt
import re as RegEx
import json
from typing import Dict, List, Set, Union

from sa_bil.core.model.connectivityGraph import ConnectivityGraph
from sa_bil.core.model.map import Map
from sa_bil.core.model.shadowTreeV2 import ShadowTreeV2
from sa_bil.core.observation.observations import Observation, Observations
from sa_bil.core.spec.spaceTime import ProjectiveSpaceTimeSet
from sa_bil.core.spec.validator import Validator
from sa_bil.core.utils.graph import GraphAlgorithms
from sa_bil.core.utils.geometry import Geometry


class Penny:
	"""
		This is the weirdest name I could use.
		This represents a penny in what Dr. Shell referred to as a stack of pennies that will track a possible pose and its state
		`state`: str -> the name of the state in the nfa graph
		`path`: List[str] -> the path in the shadow graph that has so far carried this penny over the transitions
	"""
	def __init__(self, state: str, path: List[str]):
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
		self.validator: Validator = validators[self.name]

	def __repr__(self):
		return repr(self.specifier)

	def execute(self, penny: Penny, shadowTree: ShadowTreeV2) -> Union[List[str], None]:
		"""
			Here we check whether any node in the shadow tree satisfy the validator for this node
			#### Returns
				The path that satisfied the validator function or `None`
		"""
		path = GraphAlgorithms.bfs(shadowTree, penny.path[-1], self.validator.lambdaObj.func)
		return None if path is None else path
class NFA(nx.DiGraph):
	def __init__(self, specName, states, transitions, validators):
		super().__init__()
		self._specName = specName
		self.states = states
		self.transitions = transitions
		self.validators: Dict[str, Validator] = validators
		self.START_SYMBOL = "START"
		self.TERMINAL_SYMBOL = "END"
		self._fig = None
		self.activeStates: Set[Penny] = set()
		self._buildGraph()
		self._previousCGraph: ConnectivityGraph = None

	def __repr__(self):
		return "%s.NFA" % self._specName

	def _buildGraph(self):
		if self.START_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.START_SYMBOL)
		if self.TERMINAL_SYMBOL not in self.states: raise RuntimeError("%s is not among the states." % self.TERMINAL_SYMBOL)
		self.add_nodes_from(self.states)
		for fromState in self.transitions:
			for i in range(len(self.transitions[fromState])):
				matches = RegEx.search(r"\((\(.*\)),\s+(.*)\)", self.transitions[fromState][i])
				transition = Transition(matches.group(1), self.validators)
				toState = matches.group(2)
				self.add_edge(fromState, toState, transition=transition)

	def readAll(self, envMap: Map, observations: Observations) -> Union[List[str], None]:
		if len(observations.getObservationByIndex(0).fov.sensors) == 0:
			print("Don't know how to handle no sensor yet")
			return
		fovs = [observations[o].fov for o in observations]
		shadowTree = ShadowTree(envMap, fovs, self.validators, observations.tracks)
		# TODO: test with a trajectory
		for s in shadowTree.graphs[0].shadowNodes:
			self.activeStates.add(Penny(self.START_SYMBOL, [shadowTree._generateTemporalName(s, shadowTree.graphs[0].time)]))

		# propagate pennies in shadow tree
		while len(self.activeStates) > 0:
			penny = self.activeStates.pop()
			for outEdge in self.out_edges(penny.state):
				currentState = outEdge[0]
				nextState = outEdge[1]
				transition: Transition = self.get_edge_data(currentState, nextState)["transition"]
				certificate = transition.execute(penny, shadowTree)
				if certificate is not None:
					fullCertificate = penny.path[:-1] + certificate
					if nextState == self.TERMINAL_SYMBOL:
						return fullCertificate
					self.activeStates.add(Penny(nextState, fullCertificate))
					# FIXME: We are not dealing with non-consuming edges
					# if penny in self.activeStates: self.activeStates.remove(penny) # Non-consuming pennies are not in this set
					# # While there are non-consuming transitions, we should keep doing traversing them.
					# if not transition.consuming:
					# 	if nextState == self.TERMINAL_SYMBOL:
					# 		return certificate
					# 	self.activeStates.add(Penny(nextState, nextShadowNode))
					# else: # FIXME: Testing: If the penny can't move forward, it's a dead end because I have the full ShadowTree
					# 	self.activeStates.add(Penny(nextState, nextShadowNode))
		return None

	def read(self, envMap, observation: Observation, prevObservation):
		raise "Not Implemented"

	def displayGraph(self):
		fig = plt.figure(len(GraphAlgorithms._allFigs))
		GraphAlgorithms._allFigs.add(fig)
		pos = nx.circular_layout(self)
		activeStates = { penny.state for penny in self.activeStates }
		inactiveStates = set(self.nodes) - activeStates
		nx.draw_networkx_nodes(self, pos, nodelist=activeStates, node_color="palegreen")
		nx.draw_networkx_nodes(self, pos, nodelist=inactiveStates - {"END"}, node_color="lightgrey")
		# END STATE
		nx.draw_networkx_nodes(self, pos, nodelist={"END"}, node_color="tomato" if "END" in inactiveStates else "palegreen")
		nx.draw_networkx_edges(self, pos)
		nx.draw_networkx_labels(self, pos, font_family="DejaVu Sans", font_size=10)
		edgeLabel = nx.get_edge_attributes(self, "transition")
		nx.draw_networkx_edge_labels(self, pos, edge_labels=edgeLabel)
		pennyText = [repr(penny) for penny in self.activeStates]
		plt.annotate("\n".join(pennyText), xy=(0.01, 0), xycoords="axes fraction")
		plt.axis("off")
		fig.show()
		self._fig = fig

	def killDisplayedGraph(self):
		if self._fig is not None:
			GraphAlgorithms.killDisplayedGraph(self._fig)
			self._fig = None
