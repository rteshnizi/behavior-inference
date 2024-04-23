from json import dumps, loads
from tempfile import TemporaryFile
from typing import Any, Literal, TypeAlias, cast

import networkx as nx
from networkx.drawing import nx_agraph

from rt_bi_behavior.Model.Transition import Transition
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs

StateToken: TypeAlias = dict[Literal["name", "shadowIdJson"], str]

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
		self.__spatialPredicates: dict[str, str] = {}
		"""Predicate to Symbol name map."""
		self.__tokenCounter = 0
		self.__baseDir: str = baseDir
		self.__transitionGrammarDir: str = transitionGrammarDir
		self.__grammarFileName: str = grammarFileName
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
		for p in transition.predicates: self.__spatialPredicates[p] = ""
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

	def __createToken(self, shadowIdJson: str) -> StateToken:
		token: StateToken = {
			"name": f"{self.__tokenCounter}",
			"shadowIdJson": shadowIdJson,
		}
		self.__tokenCounter += 1
		return token

	@property
	def spatialPredicates(self) -> list[str]:
		return list(self.__spatialPredicates.keys())

	def setSymbolicNameOfPredicate(self, symMap: dict[str, str]) -> None:
		"""
		:param symMap: Dictionary from symbolic name to predicate string
		:type symMap: `dict[str, str]`
		"""
		for p in symMap:
			if p in self.__spatialPredicates:
				self.__spatialPredicates[p] = symMap[p]
		for (frm, to) in self.edges:
			for predicate in self.__spatialPredicates:
				symbol = self.__spatialPredicates[predicate]
				transition = cast(Transition, self[frm][to]["transition"])
				transition.setPredicatesSymbol(predicate, symbol)
				self[frm][to]["label"] = repr(transition)
		return

	def resetTokens(self, shadowNodesStr: str) -> None:
		shadowNodes: list[str] = loads(shadowNodesStr)
		self.nodes[self.__start]["tokens"] = []
		# Ros.Logger().error(f"RESET: {repr(startState.tokens)}")
		for shadowJson in shadowNodes:
			token = self.__createToken(shadowJson)
			self.nodes[self.__start]["tokens"].append(token)
		self.nodes[self.__start]["label"] = self.__nodeLabel(self.__start, self.nodes[self.__start]["tokens"])
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

	def tokens(self) -> dict[str, list[str]]:
		d: dict[str, list[str]] = {}
		for node in self.nodes:
			d[node] = self.nodes[node]["tokens"]
		return d

	def __nodeLabel(self, nodeName: str, tokens: list[StateToken]) -> str:
		cols: list[str] = []
		for token in tokens:
			cols.append(f"<TD bgcolor='red'>{token['name']}</TD>")
		colsStr = "".join(cols)
		if len(colsStr) > 0:
			colsStr = f"<TR>{colsStr}</TR>"
		colSpan = 1 if len(cols) == 0 else len(cols)
		label = f"<<TABLE border='0' cellborder='0' cellpadding='2'><TR><TD colspan='{colSpan}'>{nodeName}</TD></TR>{colsStr}</TABLE>>" #CSpell: ignore -- cellborder
		return label

	def __prepareDot(self) -> str:
		with TemporaryFile() as f:
			aGraph = nx_agraph.to_agraph(self)
			# aGraph.graph_attr["fontname"] = "Courier"
			aGraph.draw(path=f, prog="dot", format="svg")
			f.seek(0)
			# Ros.Logger().error(str(aGraph))
			svg = f.read().decode()
			return dumps({"name": self.name, "svg": svg, "tokens": self.tokens()})

	def render(self) -> None:
		if self.__dotPublisher is None: return
		dataStr = self.__prepareDot()
		self.__dotPublisher.publish(Msgs.Std.String(data=dataStr))
		return
