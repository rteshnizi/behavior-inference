from pathlib import Path
from typing import Any, Literal, NamedTuple, cast, overload

from rt_bi_interfaces.srv import StaticReachability, SymbolResolver


class Queries(NamedTuple):
	RESOLVE_SYMBOL: str
	REACHABILITY: str

__Q = Literal["resolve", "reachability"]

class SparqlHelper:
	def __init__(self, sparqlDir: str) -> None:
		self.__sparqlDir = sparqlDir
		self.__queries = Queries(
			RESOLVE_SYMBOL=Path(self.__sparqlDir, "resolve-symbol.rq").read_text(),
			REACHABILITY=Path(self.__sparqlDir, "static-reachability.rq").read_text(),
		)
		return

	@overload
	def query(self, queryName: Literal["resolve"], req: SymbolResolver.Request) -> str: ...

	@overload
	def query(self, queryName: Literal["reachability"], req: StaticReachability.Request) -> str: ...

	def query(self, queryName: __Q, req: Any) -> str:
		if queryName == "resolve":
			symReq = cast(SymbolResolver.Request, req)
			queryStr = self.__queries.RESOLVE_SYMBOL
			return queryStr
		if queryName == "reachability":
			reachReq = cast(StaticReachability.Request, req)
			queryStr = self.__queries.RESOLVE_SYMBOL
			queryStr = queryStr.replace("?inputHasLegs", str(reachReq.include_type.legs).lower())
			queryStr = queryStr.replace("?inputHasWheels", str(reachReq.include_type.wheels).lower())
			queryStr = queryStr.replace("?inputCanSwim", str(reachReq.include_type.swim).lower())
			return queryStr
		raise ValueError(f"Unexpected query name: {queryName}")
