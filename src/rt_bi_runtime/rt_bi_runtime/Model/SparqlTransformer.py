from pathlib import Path
from typing import Any, Final, Literal, cast

from typing_extensions import deprecated

from rt_bi_commons.Base.TransitionParser import Discard, Token, TransitionParser, TransitionTransformer, Tree, UnexpectedToken
from rt_bi_commons.Utils import Ros


def _ToTitleCase(variables: list[str]) -> str:
	return ''.join([s.title() for s in variables])

class PredicateToWhere(TransitionTransformer):
	def NOT(self, _: Any) -> Any: return Discard
	def connector(self, _: Any) -> Any: return Discard
	def value(self, _: Any) -> Any: return Discard
	def test(self, _: Any) -> Any: return Discard

	def simple_expression(self, subExpressions: list[list[str]]) -> str:
		# OPTIONAL { ?regularSetId property:material/property:name ?name . }
		if len(subExpressions) != 1: raise UnexpectedToken(subExpressions, {"1 strings: variable."})
		variables = subExpressions[0]
		# ["material", "name"]
		titleCase = _ToTitleCase(variables)
		variables = [f"property:{v}" for v in variables]
		propSeq = "/".join(variables)
		# property:material/property:name
		where = f"OPTIONAL {{ ?regularSetId {propSeq} ?{titleCase} }}"
		return where

	def property_seq(self, variables: list[str]) -> list[str]:
		return variables

PREDICATE_VARNAME_PLACEHOLDER: Final = "?predicate_varname"
class PredicateToBind(TransitionTransformer):
	def EQ(self, _: Any) -> Literal["="]: return "="
	def connector(self, _: Token) -> Any: raise UnexpectedToken(_, {"No connectors are expected in a single predicate."})
	def value(self, v: list[str]) -> str: return v[0]
	def test(self, t: list[str]) -> str: return t[0]

	def simple_expression(self, subExpressions: list[str]) -> str:
		if len(subExpressions) != 3: raise UnexpectedToken(subExpressions, {"3 strings: variable, test, value"})
		return f"BIND ({subExpressions[0]} {subExpressions[1]} {subExpressions[2]} AS {PREDICATE_VARNAME_PLACEHOLDER})"

	def property_seq(self, variables: list[str]) -> str:
		titleCase = _ToTitleCase(variables)
		return f"?{titleCase}"

class PredicateToQueryStr:
	def __init__(
		self,
		baseDir: str,
		transitionGrammarDir: str,
		transitionGrammarFileName: str,
		sparqlDir: str,
	) -> None:
		super().__init__()
		self.__baseDir = baseDir
		self.__sparqlDir = sparqlDir
		self.__transitionParser = TransitionParser(baseDir, transitionGrammarDir, transitionGrammarFileName)
		return

	@deprecated("We don't use template files anymore for predicates.")
	def __regexMatchPlaceholders(self, queryName: str, placeholder: str) -> str:
		queryContent = Path(self.__baseDir, self.__sparqlDir, queryName).read_text()
		import re
		pattern = f"{placeholder}(.*){placeholder}"
		result = re.search(pattern, queryContent, re.RegexFlag.DOTALL)
		if result: return result.group(1)
		raise RuntimeError(f"The query file for variable \"{queryName}\" does not contain the required placeholder comments.")

	def __toBind(self, parseTree: Tree[str], index: int) -> tuple[str, str]:
		bindStatement = cast(str, PredicateToBind().transform(parseTree))
		if bindStatement == "": return ("", "")
		varName = f"?p_{index}"
		bindStatement = bindStatement.replace(PREDICATE_VARNAME_PLACEHOLDER, varName)
		return (bindStatement, varName)

	def __toWhere(self, parseTree: Tree[str]) -> str:
		whereClause = PredicateToWhere().transform(parseTree)
		return whereClause

	def transformPredicate(self, predicate: str, index: int) -> tuple[str, str, str]:
		# For predicates, we add the bound boolean variables rather than the sparql variable
		parseTree = self.__transitionParser.parse(predicate)
		parseTree = cast(Tree[str], parseTree)
		whereClause = self.__toWhere(parseTree)
		(varBindings, variables) = self.__toBind(parseTree, index)
		return (whereClause, variables, varBindings)
