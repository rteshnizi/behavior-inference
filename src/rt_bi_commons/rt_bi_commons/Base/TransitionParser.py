from abc import ABC
from pathlib import Path
from typing import Any, Literal, TypeVar

from lark.exceptions import UnexpectedToken
from lark.lark import Lark
from lark.lexer import Token
from lark.tree import ParseTree, Tree
from lark.visitors import Discard, Interpreter, Transformer, v_args

_ = Discard # this is here to remove the unused import warning, the goal is to just re-export
_ = Tree # this is here to remove the unused import warning, the goal is to just re-export
_ = v_args # this is here to remove the unused import warning, the goal is to just re-export
class __TransitionVisitorBase(ABC):
	__Props = TypeVar("__Props", ParseTree, list[str])
	__slots__ = [
		"expression",
		"negated_expression",
		"connector",
		"simple_expression",
		"test",
		"value",
	]
	def TRUE(self, _: str = "") -> Literal["true"]: return "true"
	def FALSE(self, _: str = "") -> Literal["false"]: return "false"
	def EQ(self, _: str = "") -> Literal["=="]: return "=="
	def NEQ(self, _: str = "") -> Literal["!="]: return "!="
	def AND(self, _: str = "") -> Literal["AND"]: return "AND"
	def OR(self, _: str = "") -> Literal["OR"]: return "OR"
	def NOT(self, _: str = "") -> Literal["NOT"]: return "NOT"
	NUMBER = str
	VARIABLE = str
	ESCAPED_STRING = str
	def property_seq(self, variables: __Props) -> __Props:
		"""Performs sanity check on variables."""
		children = variables.children if isinstance(variables, Tree) else variables
		if len(children) > 2:
			raise UnexpectedToken(variables, {"at most 2 strings"})
		return variables

class TransitionInterpreter(__TransitionVisitorBase, Interpreter[Token, Any], ABC):
	def property_seq(self, variables: ParseTree) -> list[str]:
		variables = super().property_seq(variables)
		return self.visit_children(variables)

class TransitionTransformer(__TransitionVisitorBase, Transformer[str, Any], ABC):
	expression = Transformer.__default__
	negated_expression = Transformer.__default__
	connector = Transformer.__default__
	simple_expression = Transformer.__default__
	test = Transformer.__default__
	value = Transformer.__default__
	def property_seq(self, variables: list[str]) -> list[str]:
		return super().property_seq(variables)

class TransitionParser(Lark):
	def __init__(self, baseDir: str, transitionGrammarDir: str, grammarFileName: str) -> None:
		grammarFilePath = Path(baseDir, transitionGrammarDir, grammarFileName)
		grammar = grammarFilePath.read_text()
		super().__init__(grammar, parser="lalr", transformer=Transformer()) # CSpell: ignore - lalr
