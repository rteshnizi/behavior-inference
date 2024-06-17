from collections import deque
from json import dumps, loads
from tempfile import TemporaryFile
from typing import Final, cast

import networkx as nx
from networkx.drawing import nx_agraph

from rt_bi_behavior.Model.BehaviorIGraph import BehaviorIGraph
from rt_bi_behavior.Model.State import StateWithoutHistory, TokenWithoutHistory
from rt_bi_behavior.Model.Transition import Transition, TransitionStatement
from rt_bi_commons.Shared.NodeId import NodeId
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class BehaviorAutomaton(nx.DiGraph):
	DOT_RENDER_MAX_TOKENS: Final[int] = 20
	MAX_TOKENS_WARNING: Final[int] = 2000
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
		self.name = specName
		assert start in states, f"Start state {start} is not in the set of states {states} in BA {self.name}."
		for state in accepting: assert state in states, f"Accepting state {start} is not in the set of states {states} in BA {self.name}."
		self.__start: str = start
		self.__accepting: list[str] = accepting
		self.__tokenCounter = 0
		self.__baseDir: str = baseDir
		self.__transitionGrammarDir: str = transitionGrammarDir
		self.__grammarFileName: str = grammarFileName
		self.__initializedTokens = False
		self.__buildGraph(states, transitions)
		return

	@property
	def states(self) -> dict[str, StateWithoutHistory]:
		# This is the effective structure of the NodeView class here.
		return cast(dict[str, StateWithoutHistory], self.nodes)

	def __getitem__(self, state: str) -> dict[str, Transition]:
		# This is the effective structure of the AtlasView class here.
		return cast(dict[str, Transition], super().__getitem__(state))

	def __repr__(self):
		return self.name

	def __updateStateLabel(self, state: str) -> str:
		Ros.Log(f"Updating label of {state}.")
		cols: list[str] = []
		if len(self.states[state]["tokens"]) > self.DOT_RENDER_MAX_TOKENS:
			cols.append(f"<TD bgcolor='red'>{len(self.states[state]['tokens'])}</TD>")
		else:
			for token in self.states[state]["tokens"]:
				cols.append(f"<TD bgcolor='orange'>{token['id']}</TD>")
		colsStr = "".join(cols)
		if len(colsStr) > 0:
			colsStr = f"<TR>{colsStr}</TR>"
		colSpan = 1 if len(cols) == 0 else len(cols)
		label = f"<<TABLE border='0' cellborder='0' cellpadding='2'><TR><TD colspan='{colSpan}'>{state}</TD></TR>{colsStr}</TABLE>>" #CSpell: ignore -- cellborder
		self.states[state]["label"] = label
		if state in self.__accepting and len(self.states[state]["tokens"]) > 0:
			styles = self.states[state]["style"].split(",")
			if "filled" not in styles: styles.append("filled")
			self.states[state]["style"] = ",".join(styles)
			self.states[state]["fillcolor"] = "DarkGreen"
		return label

	def __addState(self, name: str) -> None:
		starting = name == self.__start
		accepting = name in self.__accepting
		styles = ["rounded"]
		if starting: styles.append("filled")
		peripheries = 2 if accepting else 1
		tokens = []
		self.add_node(
			name,
			label=name,
			tokens=tokens,
			shape="box",
			style=",".join(styles),
			peripheries=peripheries,
		)
		self.__updateStateLabel(name)
		return

	def __addTransition(self, source: str, syntax: str, destination: str) -> None:
		statement = TransitionStatement(syntax, self.__baseDir, self.__transitionGrammarDir, self.__grammarFileName)
		self.add_edge(source, destination, label=syntax, statement=statement)
		return

	def __buildGraph(self, states: list[str], transitions: dict[str, dict[str, str]]) -> None:
		for n in states:
			self.__addState(n)
		for i in range(len(states)):
			src = states[i]
			if src not in transitions: continue
			for dst in transitions[src]:
				statementSyntax = transitions[src][dst]
				self.__addTransition(src, statementSyntax, dst)
		return

	def __createToken(self, iGraphNodeId: str | NodeId) -> TokenWithoutHistory:
		nodeId = NodeId.fromJson(iGraphNodeId) if isinstance(iGraphNodeId, str) else iGraphNodeId
		token = TokenWithoutHistory(id=f"{self.__tokenCounter}", iGraphNode=nodeId)
		self.__tokenCounter += 1
		# Ros.Log(f"New Token {token}")
		return token

	def __addToken(self, state: str, iGraphNodeId: NodeId) -> TokenWithoutHistory:
		for token in self.states[state]["tokens"]:
			if token["iGraphNode"].copy(hIndex=-1) == iGraphNodeId.copy(hIndex=-1):
				# Tokens must move forward in time. Because time is strictly increasing.
				if token["iGraphNode"].hIndex < iGraphNodeId.hIndex: token["iGraphNode"] = iGraphNodeId
				return token
		self.states[state]["tokens"].append(self.__createToken(iGraphNodeId))
		return self.states[state]["tokens"][-1]

	@property
	def initializedTokens(self) -> bool:
		return self.__initializedTokens

	@property
	def predicates(self) -> list[str]:
		d = {}
		for (frm, to, statement) in self.edges(data="statement"): # pyright: ignore[reportArgumentType]
			statement = cast(TransitionStatement, statement)
			d |= statement.predicates
		return list(d.keys())

	def reduceUncertainty(self, state: str, iGraph: BehaviorIGraph) -> None:
		tokens = self.states[state]["tokens"]
		i = 0
		while i < (len(tokens)):
			if tokens[i]["iGraphNode"] not in iGraph.nodes: # Token has expired as the node is not in history anymore
				tokens.pop(i)
				i -= 1
			i += 1
		return

	def evaluate(self, iGraph: BehaviorIGraph) -> None:
		"""
		Traverses the BA states in BFS fashion and updates their tokens.
		BFS traversal ensures tokens are pushed all the way.
		"""
		Ros.Log(128 * f"┬")
		totalTokens = 0
		for s in self.states: totalTokens += len(self.states[s]["tokens"])
		if totalTokens > self.MAX_TOKENS_WARNING: Ros.Logger().error(f"Large number of tokens: {totalTokens}")
		Ros.Log(f"Evaluating tokens of {self.name}.")
		statesToUpdate = deque([self.__start], maxlen=len(self.states))
		while len(statesToUpdate) > 0:
			fromState = statesToUpdate.popleft() # CSpell: ignore -- popleft
			self.reduceUncertainty(fromState, iGraph)
			Ros.Log(f"From State {fromState}.")
			Ros.Log(f"AFTER REDUCTION has {len(self.states[fromState]['tokens'])} tokens.")
			for toState in self[fromState]:
				statesToUpdate.append(toState)
				if len(self.states[fromState]["tokens"]) == 0: continue
				Ros.Log(f"To State {toState}")
				statement = self[fromState][toState]["statement"]
				newPositions: list[NodeId] = []
				for tokens in self.states[fromState]["tokens"]:
					for destination in iGraph.neighbors(tokens["iGraphNode"]):
						if iGraph.satisfies(destination, statement):
							token = self.__addToken(toState, destination)
							if toState in self.__accepting:
								Ros.Logger().error(f"ACCEPTED {token['iGraphNode']}")
						else:
							newPositions.append(destination)
				for nId in newPositions:
					self.__addToken(fromState, nId) # Increase Uncertainty
			self.__updateStateLabel(fromState)
		Ros.Log(128 * f"┴")
		return

	def setSymbolicNameOfPredicate(self, symbols: dict[str, str]) -> None:
		"""
		:param symMap: Dictionary from symbolic name to predicate string
		:type symMap: `dict[str, str]`
		"""
		# Update edge labels in rendering
		Ros.Log("Symbols updated in BA", symbols.items())
		for transitionSyntax in symbols:
			for (frm, to, transition) in self.edges(data="statement"): # pyright: ignore[reportArgumentType]
				transition = cast(TransitionStatement, transition)
				if transitionSyntax not in transition: continue
				transition.setPredicatesSymbol(transitionSyntax, symbols[transitionSyntax])
				self[frm][to]["label"] = repr(transition)
		return

	def __removeAllTokens(self) -> None:
		for n in self.states: self.states[n]["tokens"] = []
		return

	def resetTokens(self, iGraph: BehaviorIGraph) -> None:
		Ros.Log(f"Resetting tokens of {self.name}.")
		self.__removeAllTokens()
		self.__tokenCounter = 0
		for nodeId in iGraph.nodes:
			token = self.__createToken(nodeId)
			self.states[self.__start]["tokens"].append(token)
		self.__updateStateLabel(self.__start)
		self.__initializedTokens = True
		return

	def initFlask(self, rosNode: Ros.Node) -> None:
		(self.__dotPublisher, _) = Ros.CreatePublisher(
			rosNode,
			Msgs.Std.String,
			"/rt_bi_behavior/dot_renderer",
			callbackFunc=self.render,
			intervalSecs=2,
		)
		return

	def __tokensReportForDot(self) -> dict[str, list[dict[str, str]]]:
		d: dict[str, list[dict]] = {}
		for state in self.states:
			if len(self.states[state]["tokens"]) > self.DOT_RENDER_MAX_TOKENS:
				d[state] = [{
					"id": "===>",
					"iGraphNode": f"Has {len(self.states[state]['tokens'])} tokens...",
				}]
			else:
				d[state] = []
				for t in self.states[state]["tokens"]:
					d[state].append({
						"id": t["id"],
						"iGraphNode": repr(t["iGraphNode"]),
					})
		return d

	def __prepareDot(self) -> str:
		with TemporaryFile() as f:
			aGraph = nx_agraph.to_agraph(self)
			aGraph.draw(path=f, prog="dot", format="svg")
			f.seek(0)
			svg = f.read().decode()
			return dumps({ "name": self.name, "svg": svg, "tokens": self.__tokensReportForDot() })

	def render(self) -> None:
		if self.__dotPublisher is None: return
		dataStr = self.__prepareDot()
		self.__dotPublisher.publish(Msgs.Std.String(data=dataStr))
		# Ros.Log(f"Dot data sent for {self.name}.")
		return
