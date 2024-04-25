from json import dumps, loads
from tempfile import TemporaryFile
from typing import cast

import networkx as nx
from networkx.drawing import nx_agraph

from rt_bi_behavior.Model.RhsIGraph import RhsIGraph
from rt_bi_behavior.Model.StateToken import StateToken
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
		self.name = self.__specName
		self.__states: list[str] = states
		self.__transitions: dict[str, dict[str, str]] = transitions
		self.__start: str = start
		self.__accepting: set[str] = set(accepting)
		self.__predicatesAreSymbolizingRoundsLeft = 2
		"""Two rounds of symbols should arrive, one for spatial and one temporal predicates"""
		self.__tokenCounter = 0
		self.__baseDir: str = baseDir
		self.__transitionGrammarDir: str = transitionGrammarDir
		self.__grammarFileName: str = grammarFileName
		self.__initializedTokens = False
		self.__buildGraph()
		return

	def __repr__(self):
		return self.name

	def __addNode(self, name: str) -> None:
		starting = name == self.__start
		accepting = name in self.__accepting
		styles = ["rounded"]
		if starting: styles.append("filled")
		peripheries = 2 if accepting else 1
		tokens = []
		self.add_node(
			name,
			label=self.__nodeLabel(name, tokens),
			tokens=tokens,
			shape="box",
			style=", ".join(styles),
			peripheries=peripheries,
		)
		return

	def __addEdge(self, source: str, transitionStr: str, destination: str) -> None:
		transition = Transition(transitionStr, self.__baseDir, self.__transitionGrammarDir, self.__grammarFileName)
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

	def __createToken(self, iGraphNodeId: str | NodeId) -> StateToken:
		nodeId = NodeId.fromJson(iGraphNodeId) if isinstance(iGraphNodeId, str) else iGraphNodeId
		token = StateToken(id=f"{self.__tokenCounter}", iGraphNode=nodeId)
		self.__tokenCounter += 1
		# Ros.Log(f"New Token {token}")
		return token

	@property
	def initializedTokens(self) -> bool:
		return self.__initializedTokens

	@property
	def temporalPredicates(self) -> list[str]:
		d = {}
		for (frm, to, transition) in self.edges(data="transition"): # pyright: ignore[reportArgumentType]
			transition = cast(Transition, transition)
			d |= transition.temporalPredicates
		return list(d.keys())

	@property
	def spatialPredicates(self) -> list[str]:
		d = {}
		for (frm, to, transition) in self.edges(data="transition"): # pyright: ignore[reportArgumentType]
			transition = cast(Transition, transition)
			d |= transition.spatialPredicates
		return list(d.keys())

	def propagate(self, state: str, iGraph: RhsIGraph) -> None:
		tokens: list[StateToken] = self.nodes[state]["tokens"].copy()
		for token in tokens:
			destinations = iGraph.propagate(token["iGraphNode"])
			for destination in destinations:
				tokenPrime = self.__createToken(destination)
				self.nodes[state]["tokens"].append(tokenPrime)
		return

	def evaluate(self, iGraph: RhsIGraph) -> None:
		Ros.Log(f"Evaluating tokens of {self.name}.")
		for state in self.nodes:
			state = cast(str, state)
			tokens: list[StateToken] = self.nodes[state]["tokens"]
			if len(tokens) == 0: continue
			for (_, toState, transition) in self.out_edges(state, data="transition"): # pyright: ignore[reportArgumentType]
				toState = cast(str, toState)
				transition = cast(Transition, transition)
				nodeFilter = lambda n: iGraph.destinationFilter(transition, n)
				destinations: list[NodeId] = list(nx.subgraph_view(iGraph, filter_node=nodeFilter, filter_edge=iGraph.removeAllFilter).nodes)
				if len(destinations) > 0:
					i = 0
					while i < (len(tokens)):
						token = tokens[i]
						source = token["iGraphNode"]
						if source not in iGraph.nodes: # Token has expired as the node is not in history anymore
							tokens.pop(i)
							i -= 1
						found = iGraph.path(source, destinations)
						for destination in found:
							self.nodes[toState]["tokens"].append(self.__createToken(destination))
						i += 1
				self.propagate(state, iGraph)
		return

	def setSymbolicNameOfPredicate(self, symbols: dict[str, str]) -> None:
		"""
		:param symMap: Dictionary from symbolic name to predicate string
		:type symMap: `dict[str, str]`
		"""
		# Update edge labels in rendering
		Ros.Log("Symbols updated in BA", symbols.items())
		for transitionSyntax in symbols:
			for (frm, to, transition) in self.edges(data="transition"): # pyright: ignore[reportArgumentType]
				transition = cast(Transition, transition)
				if transitionSyntax not in transition: continue
				transition.setPredicatesSymbol(transitionSyntax, symbols[transitionSyntax])
			self[frm][to]["label"] = repr(transition)
			Ros.Log("Transition updated to", [transition])
		if self.__predicatesAreSymbolizingRoundsLeft > 0: self.__predicatesAreSymbolizingRoundsLeft -= 1
		return

	def __removeAllTokens(self) -> None:
		for n in self.nodes: self.nodes[n]["tokens"] = []
		return

	def resetTokens(self, iGraph: RhsIGraph) -> None:
		if self.__predicatesAreSymbolizingRoundsLeft > 0: return # Still awaiting symbols
		Ros.Log(f"Resetting tokens of {self.name}.")
		self.__removeAllTokens()
		self.__tokenCounter = 0
		for nodeId in iGraph.nodes:
			token = self.__createToken(nodeId)
			self.nodes[self.__start]["tokens"].append(token)
		self.nodes[self.__start]["label"] = self.__nodeLabel(self.__start, self.nodes[self.__start]["tokens"])
		self.__initializedTokens = True
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

	def __tokensPerState(self) -> dict[str, list[StateToken]]:
		d: dict[str, list[StateToken]] = {}
		for node in self.nodes:
			d[node] = self.nodes[node]["tokens"]
		return d

	def __nodeLabel(self, nodeName: str, tokens: list[StateToken]) -> str:
		cols: list[str] = []
		for token in tokens:
			cols.append(f"<TD bgcolor='red'>{token['id']}</TD>")
		colsStr = "".join(cols)
		if len(colsStr) > 0:
			colsStr = f"<TR>{colsStr}</TR>"
		colSpan = 1 if len(cols) == 0 else len(cols)
		label = f"<<TABLE border='0' cellborder='0' cellpadding='2'><TR><TD colspan='{colSpan}'>{nodeName}</TD></TR>{colsStr}</TABLE>>" #CSpell: ignore -- cellborder
		return label

	def __prepareDot(self) -> str:
		with TemporaryFile() as f:
			aGraph = nx_agraph.to_agraph(self)
			aGraph.draw(path=f, prog="dot", format="svg")
			f.seek(0)
			svg = f.read().decode()
			return dumps({"name": self.name, "svg": svg, "tokens": self.__tokensPerState()})

	def render(self) -> None:
		if self.__dotPublisher is None: return
		dataStr = self.__prepareDot()
		self.__dotPublisher.publish(Msgs.Std.String(data=dataStr))
		return
