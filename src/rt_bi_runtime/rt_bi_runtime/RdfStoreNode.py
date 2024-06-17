from json import dumps
from pathlib import Path
from typing import Any, Literal

from ament_index_python.packages import get_package_share_directory

from rt_bi_commons.Base.ColdStartableNode import ColdStartPayload
from rt_bi_commons.Base.DataDictionaryNode import DataDictionaryNode
from rt_bi_commons.RosParamParsers.AtomicParsers import StrParser
from rt_bi_commons.RosParamParsers.ParamParser import ParserBase
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.srv import SpaceTime
from rt_bi_runtime import package_name
from rt_bi_runtime.Model.FusekiInterface import FusekiInterface
from rt_bi_runtime.Model.SparqlTransformer import PredicateToQueryStr

QueryTemplates = Literal[
	"sparql_channel",
	"sparql_geometry",
	"sparql_intervals",
	"sparql_sets",
]
_Parameters = Literal[
	"fuseki_server",
	"rdf_dir",
	"rdf_store",
	"sparql_dir",
	"placeholder_bind",
	"placeholder_ids",
	"placeholder_order",
	"placeholder_select",
	"placeholder_where",
	"transition_grammar_dir",
	"transition_grammar_file",
] | QueryTemplates
# FIXME: Upload the rdf data at the startup via cold start.
class RdfStoreNode(DataDictionaryNode[_Parameters]):
	def __init__(self, **kwArgs) -> None:
		parsers: dict[_Parameters, ParserBase[_Parameters, Any, Any]] = {
			"fuseki_server": StrParser[_Parameters](self, "fuseki_server"),
			"rdf_dir": StrParser[_Parameters](self, "rdf_dir"),
			"rdf_store": StrParser[_Parameters](self, "rdf_store"),
			"sparql_dir": StrParser[_Parameters](self, "sparql_dir"),
			"sparql_channel": StrParser[_Parameters](self, "sparql_channel"),
			"sparql_geometry": StrParser[_Parameters](self, "sparql_geometry"),
			"sparql_intervals": StrParser[_Parameters](self, "sparql_intervals"),
			"sparql_sets": StrParser[_Parameters](self, "sparql_sets"),
			"placeholder_bind": StrParser[_Parameters](self, "placeholder_bind"),
			"placeholder_ids": StrParser[_Parameters](self, "placeholder_ids"),
			"placeholder_order": StrParser[_Parameters](self, "placeholder_order"),
			"placeholder_where": StrParser[_Parameters](self, "placeholder_where"),
			"placeholder_select": StrParser[_Parameters](self, "placeholder_select"),
			"transition_grammar_dir": StrParser[_Parameters](self, "transition_grammar_dir"),
			"transition_grammar_file": StrParser[_Parameters](self, "transition_grammar_file"),
		}
		newKw = { "node_name": "dd_rdf", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		super().__init__(parsers, **newKw)
		self.__baseDir = get_package_share_directory(package_name)
		self.__httpInterface = FusekiInterface(self, self["fuseki_server"][0], self["rdf_store"][0])
		self.__predicateToIndex: dict[str, int] = {}
		RtBiInterfaces.createSpaceTimeService(self, self.__onSpaceTimeRequest)

	def __joinList(self, l: list[str], separator: str) -> str:
		if len(l) == 0: return ""
		l = [s.strip() for s in l if s.strip() != ""]
		l = list(dict.fromkeys(l)) # Remove duplicates
		return separator.join(l)

	def __createFilterStatement(self, variables: list[str]) -> str:
		variables = list(filter(lambda s: s.startswith("?p_"), variables))
		condition = self.__joinList(variables, " || ")
		return f"FILTER ({condition})"

	def __templateParamToFileParam(self, param: QueryTemplates) -> str:
		if (
			param == "sparql_sets" or
			param == "sparql_channel" or
			param == "sparql_geometry" or
			param == "sparql_intervals"
		):
			return self[param][0]
		raise RuntimeError(f"Unexpected template file parameter name: {param}")

	def __fillTemplate(self, templateParam: QueryTemplates, ids: list[str], whereClauses: list[str], variables: list[str], binds: list[str], orders: list[str]) -> str:
		fileName = self.__templateParamToFileParam(templateParam)
		sparql = Path(self.__baseDir, self["sparql_dir"][0], fileName).read_text()
		sparql = sparql.replace(self["placeholder_select"][0], self.__joinList(variables, " "))
		ids = [f"<{iri}>" for iri in ids]
		sparql = sparql.replace(self["placeholder_ids"][0], self.__joinList(ids, "\n\t\t"))
		sparql = sparql.replace(self["placeholder_where"][0], self.__joinList(whereClauses, "\n\t"))
		sparql = sparql.replace(self["placeholder_bind"][0], self.__joinList(binds, "\n\t"))
		sparql = sparql.replace(self["placeholder_order"][0], self.__joinList(orders, " "))
		return sparql

	def __onSpaceTimeRequest(self, req: SpaceTime.Request, res: SpaceTime.Response) -> SpaceTime.Response:
		payload = ColdStartPayload(req.json_payload)
		if req.query_name == "sets": res = self.__setQuery(payload, res)
		else: raise RuntimeError(f"Unexpected query name: {req.query_name}")
		return res

	def __setQuery(self, payload: ColdStartPayload, res: SpaceTime.Response) -> SpaceTime.Response:
		predicateMapping: dict[str, str] = {}
		ids: list[str] = []
		whereClauses: list[str] = []
		variables: list[str] = []
		binds: list[str] = []
		orders: list[str] = []
		sparqlXfmr = PredicateToQueryStr(self.__baseDir, self["transition_grammar_dir"][0], self["transition_grammar_file"][0])
		for predicate in payload.predicates:
			if predicate not in self.__predicateToIndex: self.__predicateToIndex[predicate] = len(self.__predicateToIndex)
			(extractedSelector, extractedVars, extractedBindings) = sparqlXfmr.transformPredicate(predicate, self.__predicateToIndex[predicate])
			if extractedVars == "" and extractedSelector == "": continue
			predicateMapping[predicate] = extractedVars
			variables.append(extractedVars)
			whereClauses.append(extractedSelector)
			binds.append(extractedBindings)
		binds.append(self.__createFilterStatement(variables))
		sparql = self.__fillTemplate("sparql_sets", ids, whereClauses, variables, binds, orders)
		res.json_predicate_symbols = dumps(predicateMapping)
		msgsByTypeById = self.__httpInterface.fetchSets(sparql)
		setMsgs = self.__queryById("sparql_geometry", msgsByTypeById["static"] | msgsByTypeById["dynamic"])
		setMsgs |= self.__queryById("sparql_intervals", msgsByTypeById["dynamic"] | msgsByTypeById["temporal"])
		# setMsgs += self.__queryById(msgsByTypeAndId["affine"])
		Ros.ConcatMessageArray(res.sets, list(setMsgs.values()))
		return res

	def __regexMatchPlaceholders(self, templateParam: QueryTemplates, markup: str = "") -> str:
		"""
		Opens a query file associated with the given name and returns the text.
		If a `markup` string is given, the portion of the text surrounded between the markup text is return.
		"""
		fileName = self.__templateParamToFileParam(templateParam)
		queryContent = Path(self.__baseDir, self["sparql_dir"][0], fileName).read_text()
		if markup == "": return queryContent
		import re
		pattern = f"{markup}(.*){markup}"
		result = re.search(pattern, queryContent, re.RegexFlag.DOTALL)
		if result: return result.group(1)
		raise RuntimeError(f"The query file for variable \"{templateParam}\" does not contain the required placeholder comments.")

	def __queryById(self, templateParam: QueryTemplates, msgsById: dict[str, Msgs.RtBi.RegularSet]) -> dict[str, Msgs.RtBi.RegularSet]:
		if templateParam == "sparql_sets":
			raise RuntimeError(f"The template {templateParam} query doesn't have a placeholder for IDs")
		ids: list[str] = list(msgsById.keys())
		sparql = self.__fillTemplate(templateParam, ids, [], [], [], [])
		if templateParam == "sparql_geometry": return self.__httpInterface.fetchGeometryById(sparql, msgsById)
		if templateParam == "sparql_intervals": return self.__httpInterface.fetchIntervalsById(sparql, msgsById)
		raise RuntimeError(f"Unexpected template query param: {templateParam}")

	def render(self) -> None:
		return super().render()

def main(args=None) -> None:
	return RdfStoreNode.Main(args)

if __name__ == "__main__":
	main()
