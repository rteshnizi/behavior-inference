from pathlib import Path
from typing import Any, Final, Literal, cast

from rt_bi_commons.Base.TransitionParser import Discard, Token, TransitionParser, TransitionTransformer, Tree, UnexpectedToken
from rt_bi_commons.Utils import Ros


class PredicateToQueryName(TransitionTransformer):
	def NOT(self, _: Any) -> Any: return Discard
	def connector(self, _: Any) -> Any: return Discard
	def value(self, _: Any) -> Any: return Discard
	def test(self, _: Any) -> Any: return Discard

	def simple_expression(self, subExpressions: list[str]) -> str:
		if len(subExpressions) != 2: raise UnexpectedToken(subExpressions, {"2 strings: namespace, variable."})
		if subExpressions[0] == "T": return ""
		return subExpressions[1]

	def property_seq(self, variables: list[str]) -> str:
		return variables[0]

class PredicateToNamespace(TransitionTransformer):
	def NAMESPACE(self, token: Token) -> str:
		if token == "S": return "rt_bi:RegularSpace"
		if token == "T": return "rt_bi:RegularTime"
		raise UnexpectedToken(token, {"S", "T"})
	def connector(self, _: Any) -> Any: return Discard
	def property_seq(self, _: Any) -> Any: return Discard
	def test(self, _: Any) -> Any: return Discard
	def value(self, _: Any) -> Any: return Discard

	def simple_expression(self, subExpressions: list[str]) -> str:
		if len(subExpressions) != 1: raise UnexpectedToken(subExpressions, {"A single namespace."})
		return subExpressions[0]

PREDICATE_VARNAME: Final = "?predicate_varname"
class PredicateToVariable(TransitionTransformer):
	def EQ(self, _: Any) -> Literal["="]: return "="
	def connector(self, _: Token) -> Any: raise UnexpectedToken(_, {"No connectors are expected in a single predicate."})
	def value(self, v: list[str]) -> str: return v[0]
	def test(self, t: list[str]) -> str: return t[0]

	def simple_expression(self, subExpressions: list[str]) -> str:
		if len(subExpressions) != 4: raise UnexpectedToken(subExpressions, {"4 strings: namespace, variable, test, value"})
		if subExpressions[0] == "T": return ""
		return f"BIND ({subExpressions[1]} {subExpressions[2]} {subExpressions[3]} AS {PREDICATE_VARNAME})"

	def property_seq(self, variables: list[str]) -> str:
		return f"?{variables[-1]}"


class PredicateToQueryStr:
	def __init__(self, baseDir: str, transitionGrammarDir: str, transitionGrammarFileName: str, sparqlDir: str, queryFiles: list[tuple[str, str]]) -> None:
		super().__init__()
		self.__baseDir = baseDir
		self.__sparqlDir = sparqlDir
		self.__queryFiles: dict[str, str] = { pair[0]: pair[1] for pair in queryFiles }
		self.__transitionParser = TransitionParser(baseDir, transitionGrammarDir, transitionGrammarFileName)
		return

	def __toSelector(self, parsedPred: Tree[str], tagStart: str, tagEnd: str) -> str:
		queryName = cast(str, PredicateToQueryName().transform(parsedPred))
		selector = self.selector(queryName, tagStart, tagEnd)
		namespace = cast(str, PredicateToNamespace().transform(parsedPred))
		selector = selector.replace("?namespace", namespace)
		return selector

	def __toBind(self, parsedPred: Tree[str], index: int) -> tuple[str, str]:
		bindStatement = cast(str, PredicateToVariable().transform(parsedPred))
		if bindStatement == "": return ("", "")
		varName = f"?p_{index}"
		bindStatement = bindStatement.replace(PREDICATE_VARNAME, varName)
		return (bindStatement, varName)

	def transform(self, predicate: str, index: int, tagStart: str, tagEnd: str) -> tuple[str, str, str]:
		parsedPred = self.__transitionParser.parse(predicate)
		parsedPred = cast(Tree[str], parsedPred)
		selector = self.__toSelector(parsedPred, tagStart, tagEnd)
		(bindStatement, varName) = self.__toBind(parsedPred, index)
		return (varName, selector, bindStatement)

	def selector(self, queryName: str, tagStart: str, tagEnd: str) -> str:
		if queryName == "": return ""
		if queryName not in self.__queryFiles: raise KeyError(f"Unexpected variable name: {queryName}")
		selector = Path(self.__baseDir, self.__sparqlDir, self.__queryFiles[queryName]).read_text()
		import re
		pattern = f"{tagStart}(.*){tagEnd}"
		result = re.search(pattern, selector, re.RegexFlag.DOTALL)
		if result:
			selector = result.group(1)
			return selector
		raise RuntimeError(f"The query file for variable \"{queryName}\" does not contain the required comment markers.")

	@property
	def polygonVarNames(self) -> str:
		return "?polygonId ?vertInd ?x ?y"
