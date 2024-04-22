from typing import cast

from rt_bi_commons.Base.TransitionParser import ParseTree, TransitionInterpreter, TransitionParser, UnexpectedToken
from rt_bi_commons.Utils import Ros


class PredicateCollector(TransitionInterpreter):
	def __init__(self) -> None:
		super().__init__()
		self.__allPredicates: set[str] = set()

	@property
	def allPredicates(self) -> set[str]:
		return self.__allPredicates

	def simple_expression(self, tree: ParseTree) -> str:
		children: list[str] = self.visit_children(tree)
		namespace: str = cast(str, children[0])
		property_seq: list[str] = cast(list[str], children[1])
		test: str = cast(str, children[2])
		value: str = cast(str, children[3])
		interpretationStr = f"{namespace}[{property_seq} {test} {value}]"
		self.__allPredicates.add(interpretationStr)
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

class Transition:
	def __init__(self, transitionStr: str, baseDir: str, transitionGrammarDir: str, grammarFileName: str) -> None:
		self.__str: str = transitionStr
		self.__symStr: str = transitionStr
		tree = TransitionParser(baseDir, transitionGrammarDir, grammarFileName).parse(transitionStr)
		self.__predCollector = PredicateCollector()
		self.__predCollector.visit(tree)
		return

	def __str__(self) -> str:
		return self.__str

	def __repr__(self) -> str:
		return self.__symStr

	def __hash__(self) -> int:
		return hash(self.__str)

	def __eq__(self, other: "Transition") -> bool:
		return self.__str == other.__str

	@property
	def predicates(self) -> set[str]:
		return self.__predCollector.allPredicates


	def setPredicatesSymbol(self, predicate: str, symbol: str) -> None:
		if predicate == "" or symbol == "": return
		self.__symStr = self.__symStr.replace(predicate, symbol)
		Ros.Logger().error(f"Predicate = {predicate}, Syms = {symbol}")
		Ros.Logger().error(f"STR = {self.__str}, SYM = {self.__symStr}")
		return
