from pathlib import Path

from typing_extensions import Any, Final, Literal, LiteralString, NamedTuple, cast, overload

from rt_bi_commons.Utils.Ros import Logger
from rt_bi_interfaces.msg import SpaceSpec, Traversability
from rt_bi_interfaces.srv import SpaceTime


class _QueriesStrings(NamedTuple):
	RESOLVE_SYMBOL: str

_Q = Literal["resolve"]
_FILTER_PLACEHOLDER_MARKER: Final = "true #### FILTER PLACEHOLDER ####"
class SparqlHelper:
	def __init__(self, sparqlDir: str) -> None:
		self.__sparqlDir = sparqlDir
		self.__queries = _QueriesStrings(
			RESOLVE_SYMBOL=Path(self.__sparqlDir, "resolve-symbol.rq").read_text(),
		)
		return

	def __stripFilter(self, filterStr: str) -> str:
		stripped = " && ".join([s for s in filterStr.split("&&") if s.strip()])
		return stripped

	def __appendExists(self, filterStr: str, statements: str) -> str:
		if statements == "": return filterStr
		return f"{filterStr} EXISTS {{ {statements} }} &&"

	def __stripVarName(self, propName: str) -> str:
		return propName.strip().strip('?')

	def __strSparqlFilter(self, varName: str, propName: str, val: str) -> str:
		if isinstance(val, str) and val != "":
			return f"?{self.__stripVarName(varName)} {propName} \"{val}\" ."
		return ""

	def __boolSparqlFilter(self, varName: str, propName: str, val: str) -> str:
		if isinstance(val, str):
			vLower = val.lower()
			if vLower == Traversability.TRUE or vLower == Traversability.FALSE:
				return f"?{self.__stripVarName(varName)} {propName} {vLower} ."
		return ""

	def __traversabilityToSparqlFilter(self, trv: Traversability) -> str:
		subFilters: list[str] = []
		subFilters.append(self.__boolSparqlFilter("traversability", "world_props:legs", trv.legs))
		subFilters.append(self.__boolSparqlFilter("traversability", "world_props:wheels", trv.wheels))
		subFilters.append(self.__boolSparqlFilter("traversability", "world_props:swims", trv.swim))
		mergedSubFilters = " ".join([s for s in subFilters if s.strip()]).strip()
		if mergedSubFilters == "": return ""
		return f"?regularSpaceId world_props:traversability ?traversability . GRAPH tower_bridge:traversabilities {{ {mergedSubFilters} }}"

	def __specToSparqlFilter(self, spec: SpaceSpec) -> str:
		filterStr = ""
		filterStr = self.__appendExists(filterStr, self.__strSparqlFilter("regularSpaceId", "world_props:name", spec.name))
		filterStr = self.__appendExists(filterStr, self.__strSparqlFilter("regularSpaceId", "world_props:color", spec.color))
		if isinstance(spec.traversability, Traversability):
			filterStr = self.__appendExists(filterStr, self.__traversabilityToSparqlFilter(spec.traversability))
		return self.__stripFilter(filterStr)

	@overload
	def queryStr(self, queryName: Literal["resolve"], req: SpaceTime.Request) -> str: ...

	@overload
	def queryStr(self, queryName: LiteralString, req: Any) -> str: ...

	def queryStr(self, queryName: _Q | LiteralString, req: Any) -> str:
		if queryName == "resolve":
			reqST = cast(SpaceTime.Request, req)
			filterStr = self.__specToSparqlFilter(reqST.filter)
			queryStr = self.__queries.RESOLVE_SYMBOL.replace(_FILTER_PLACEHOLDER_MARKER, filterStr)
			return queryStr
		raise ValueError(f"Unexpected query name: {queryName}")
