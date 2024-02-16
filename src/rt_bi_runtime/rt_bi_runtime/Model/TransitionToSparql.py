from pathlib import Path

from lark.exceptions import UnexpectedToken
from lark.lark import Lark
from lark.lexer import Token
from lark.visitors import Transformer, v_args

from rt_bi_commons.Utils import Ros


class TransitionParser(Lark):
	def __init__(self, grammarFilePath: Path) -> None:
		grammar = grammarFilePath.read_text()
		super().__init__(grammar, parser="lalr", transformer=_ToSparql()) # CSpell: ignore - lalr

class _ToSparql(Transformer[str, str]):
	TRAVERSABILITY = "traversability"
	TRUE = str
	FALSE = str
	NUMBER = str
	EQ = str
	VARIABLE = str
	NAMESPACE = str
	def AND(self, _: Token) -> str: return "&&"
	def OR(self, _: Token) -> str: return "||"
	def NOT(self, _: Token) -> str: return "NOT"
	def ESCAPED_STRING(self, token: Token) -> str: return f"{token}"

	def expression(self, children: list[str]) -> str:
		"""Wrap expressions in parentheses."""
		joinedSubExpr = " ".join(children)
		return f"({joinedSubExpr})"

	@v_args(inline=True)
	def negated_expression(self, neg: Token, simple_expression: Token) -> str:
		return f"{neg} {simple_expression}"

	@v_args(inline=True)
	def simple_expression(self, namespace: Token, property_seq: list[str], test: str, value: str) -> str:
		"""Every `simple_expression` turns into one `EXISTS` sparql function call."""
		sparql = ""
		if namespace == "T": return sparql
		if test != "==":
			raise UnexpectedToken(test, "==")
		if len(property_seq) > 2:
			raise UnexpectedToken(property_seq, "at most 2 strings")
		elif len(property_seq) > 1:
			if property_seq[0] == self.TRAVERSABILITY: sparql = self.__traversabilitySparqlFilter(property_seq[1], value)
			else: raise UnexpectedToken(property_seq[0], self.TRAVERSABILITY)
		elif len(property_seq) == 1:
			sparql = self.__sparqlFilter("regularSpaceId", property_seq[0], value)
		return self.__wrapInExists(sparql)

	def property_seq(self, variables: list[str]) -> list[str]:
		return variables

	@v_args(inline=True)
	def test(self, operator: str) -> str:
		return operator

	@v_args(inline=True)
	def value(self, v: str) -> str:
		return v

	def __stripFilter(self, filterStr: str) -> str:
		stripped = filterStr.strip(" &|")
		# stripped = " && ".join([s for s in filterStr.split("&&") if s.strip()])
		return stripped

	def __wrapInExists(self, statement: str) -> str:
		if statement == "": return ""
		return f"EXISTS {{ {self.__stripFilter(statement)} }}"

	def __stripVarName(self, propName: str) -> str:
		return propName.strip().strip('?')

	def __sparqlFilter(self, namespace: str, propName: str, val: str) -> str:
		if isinstance(val, str) and val != "":
			return f"?{self.__stripVarName(namespace)} world_props:{propName} {val} ."
		return ""

	def __traversabilitySparqlFilter(self, propName: str, val: str) -> str:
		traversabilityMatcher = self.__sparqlFilter("regularSpaceId", self.TRAVERSABILITY, f"?{self.TRAVERSABILITY}")
		subFilter = self.__sparqlFilter(self.TRAVERSABILITY, propName, val)
		if subFilter == "": return ""
		return f"{traversabilityMatcher} GRAPH tower_bridge:traversabilities {{ {subFilter} }}"

	# from rt_bi_interfaces.msg import SpaceSpec, Traversability
	# def __strSparqlFilter(self, varName: str, propName: str, val: str) -> str:
	# 	if isinstance(val, str) and val != "":
	# 		return f"?{self.__stripVarName(varName)} {propName} \"{val}\" ."
	# 	return ""

	# def __boolSparqlFilter(self, varName: str, propName: str, val: str) -> str:
	# 	if isinstance(val, str):
	# 		vLower = val.lower()
	# 		if vLower == Traversability.TRUE or vLower == Traversability.FALSE:
	# 			return f"?{self.__stripVarName(varName)} {propName} {vLower} ."
	# 	return ""

	# def __traversabilityToSparqlFilter(self, trv: Traversability) -> str:
	# 	subFilters: list[str] = []
	# 	subFilters.append(self.__boolSparqlFilter("traversability", "world_props:legs", trv.legs))
	# 	subFilters.append(self.__boolSparqlFilter("traversability", "world_props:wheels", trv.wheels))
	# 	subFilters.append(self.__boolSparqlFilter("traversability", "world_props:swims", trv.swim))
	# 	mergedSubFilters = " ".join([s for s in subFilters if s.strip()]).strip()
	# 	if mergedSubFilters == "": return ""
	# 	return f"?regularSpaceId world_props:traversability ?traversability . GRAPH tower_bridge:traversabilities {{ {mergedSubFilters} }}"

	# def __specToSparqlFilter(self, spec: SpaceSpec) -> str:
	# 	filterStr = ""
	# 	filterStr = self.__wrapInExists(filterStr, self.__strSparqlFilter("regularSpaceId", "world_props:name", spec.name))
	# 	filterStr = self.__wrapInExists(filterStr, self.__strSparqlFilter("regularSpaceId", "world_props:color", spec.color))
	# 	if isinstance(spec.traversability, Traversability):
	# 		filterStr = self.__wrapInExists(filterStr, self.__traversabilityToSparqlFilter(spec.traversability))
	# 	return self.__stripFilter(filterStr)
