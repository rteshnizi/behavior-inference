from typing import Callable, Literal, cast

from rt_bi_commons.Base.TransitionParser import ParseTree, TransitionInterpreter, TransitionParser, TransitionTransformer, UnexpectedToken, v_args
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros


class PredicateCollector(TransitionInterpreter):
	def __init__(self, simpleExpRebuildFn: Callable[[str, str, str, str], str]) -> None:
		super().__init__()
		self.__spatialPredicates: set[str] = set()
		self.__temporalPredicates: set[str] = set()
		self.__simpleExpRebuildFn: Callable[[str, str, str, str], str] = simpleExpRebuildFn

	@property
	def spatialPredicates(self) -> set[str]:
		return self.__spatialPredicates

	@property
	def temporalPredicates(self) -> set[str]:
		return self.__temporalPredicates

	def simple_expression(self, tree: ParseTree) -> str:
		children: list[str] = self.visit_children(tree)
		namespace: str = cast(str, children[0])
		property_seq: str = cast(str, children[1])
		test: str = cast(str, children[2])
		value: str = cast(str, children[3])
		interpretationStr = self.__simpleExpRebuildFn(namespace, property_seq, test, value)
		if namespace == "S": self.__spatialPredicates.add(interpretationStr)
		elif namespace == "T": self.__temporalPredicates.add(interpretationStr)
		else: raise UnexpectedToken(tree.children[0], {"T", "S"})
		return interpretationStr

	def property_seq(self, tree: ParseTree) -> str:
		children = super().property_seq(tree)
		return ".".join(children)

	def test(self, tree: ParseTree) -> str:
		self.visit_children(tree)
		if tree.children[0] == self.EQ(): return self.EQ()
		if tree.children[0] == self.NEQ(): return self.NEQ()
		raise UnexpectedToken(tree.children[0], {self.EQ, self.NEQ})

	def value(self, tree: ParseTree) -> str:
		children: list[str] = self.visit_children(tree)
		return children[0]

class TransitionEvaluator(TransitionTransformer):
	def __init__(self, predicateSymbolMap: dict[str, str], predicates: Predicates, simpleExpRebuildFn: Callable[[str, str, str, str], str]) -> None:
		super().__init__()
		self.__simpleExpRebuildFn = simpleExpRebuildFn
		self.__symbolMap = predicateSymbolMap
		"""Map from Transition Syntax to symbolic name."""
		self.__predicates = predicates
		"""Map from symbolic name to boolean value."""
		return

	def AND(self, _: str = "") -> Literal["and"]: return "and"
	def OR(self, _: str = "") -> Literal["or"]: return "or"
	def NOT(self, _: str = "") -> Literal["not"]: return "not"

	def expression(self, children: list[str]) -> bool:
		fullStr = " ".join(children)
		try:
			val = eval(fullStr)
			if isinstance(val, bool): return val
			else: raise ValueError(f"The evaluation result for {fullStr} is not a boolean: {val}")
		except Exception as e:
			raise e

	def connector(self, children: list[str]) -> str:
		return children[0]

	@v_args(tree=True)
	def simple_expression(self, tree: ParseTree) -> str:
		namespace: str = cast(str, tree.children[0])
		property_seq: str = cast(str, tree.children[1])
		test: str = cast(str, tree.children[2])
		value: str = cast(str, tree.children[3])
		transitionSyntax = self.__simpleExpRebuildFn(namespace, property_seq, test, value)
		if transitionSyntax not in self.__symbolMap: raise KeyError(f"{transitionSyntax} does not have a symbol.")
		sym = self.__symbolMap[transitionSyntax]
		if sym not in self.__predicates:
			Ros.Log(f"Value for {transitionSyntax} is not given. Replacing it with False")
			strVal = "False"
		else:
			strVal = str(self.__predicates[sym])
		return strVal

	def property_seq(self, children: list[str]) -> str:
		children = super().property_seq(children)
		return ".".join(children)

	def test(self, children: list[str]) -> str:
		return children[0]

	def value(self, children: list[str]) -> str:
		return children[0]

class Transition:
	def __init__(self, transitionStr: str, baseDir: str, transitionGrammarDir: str, grammarFileName: str) -> None:
		self.__str: str = transitionStr
		self.__symStr: str = transitionStr
		self.__parseTree = TransitionParser(baseDir, transitionGrammarDir, grammarFileName).parse(transitionStr)
		predCollector = PredicateCollector(self.__simpleExpRebuildFn)
		predCollector.visit(self.__parseTree)
		self.spatialPredicates: dict[str, str] = { p: "" for p in predCollector.spatialPredicates }
		"""Map from Transition Syntax to symbolic name."""
		self.temporalPredicates: dict[str, str] = { p: "" for p in predCollector.temporalPredicates }
		"""Map from Transition Syntax to symbolic name."""
		return

	def __simpleExpRebuildFn(self, namespace: str, property_seq: str, test: str, value: str) -> str:
		return f"{namespace}[{property_seq} {test} {value}]"

	def __str__(self) -> str:
		return self.__str

	def __repr__(self) -> str:
		return self.__symStr

	def __hash__(self) -> int:
		return hash(self.__str)

	def __eq__(self, other: "Transition") -> bool:
		return self.__str == other.__str

	def __contains__(self, transitionSyntax: str) -> bool:
		if transitionSyntax in self.spatialPredicates: return True
		if transitionSyntax in self.temporalPredicates: return True
		return False

	def setPredicatesSymbol(self, transitionSyntax: str, symbol: str) -> None:
		if transitionSyntax == "" or symbol == "": return
		if transitionSyntax in self.spatialPredicates: self.spatialPredicates[transitionSyntax] = symbol
		if transitionSyntax in self.temporalPredicates: self.temporalPredicates[transitionSyntax] = symbol
		self.__symStr = self.__symStr.replace(transitionSyntax, symbol)
		return

	def evaluate(self, predicates: Predicates) -> bool:
		evaluator = TransitionEvaluator(
			self.spatialPredicates | self.temporalPredicates,
			predicates,
			self.__simpleExpRebuildFn,
		)
		val = evaluator.transform(self.__parseTree) # pyright: ignore[reportArgumentType]
		return val
