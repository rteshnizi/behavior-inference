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
		self.transitionStr: str = transitionStr
		tree = TransitionParser(baseDir, transitionGrammarDir, grammarFileName).parse(transitionStr)
		self.__predCollector = PredicateCollector()
		self.__predCollector.visit(tree)
		return

	def __repr__(self) -> str:
		return f"{self.transitionStr}"

	def __hash__(self) -> int:
		return hash(self.transitionStr)

	def __eq__(self, other: "Transition") -> bool:
		return self.transitionStr == other.transitionStr

	@property
	def predicates(self) -> set[str]:
		return self.__predCollector.allPredicates
