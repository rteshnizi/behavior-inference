from pathlib import Path
from typing import Any, Literal

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.ColdStartableNode import ColdStartPayload
from rt_bi_commons.Base.DataDictionaryNode import DataDictionaryNode
from rt_bi_commons.RosParamParsers.AtomicParsers import StrParser
from rt_bi_commons.RosParamParsers.JsonParser import ListJsonParser
from rt_bi_commons.RosParamParsers.ParamParser import ParserBase
from rt_bi_commons.RosParamParsers.ReferenceParser import ListReferenceParser
from rt_bi_commons.Shared.Pose import CoordsList
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_core.Temporal.TimeInterval import TimeInterval
from rt_bi_interfaces.srv import SpaceTime
from rt_bi_runtime import package_name
from rt_bi_runtime.DataStore.FusekiInterface import FusekiInterface
from rt_bi_runtime.DataStore.SparqlTransformer import PredicateToQueryStr

_Parameters = Literal[
	"fuseki_server",
	"rdf_dir",
	"rdf_store",
	"sparql_dir",
	"sparql_query_template",
	"sparql_var_selectors",
	"placeholder_bind",
	"placeholder_order_by",
	"placeholder_selector_end",
	"placeholder_selector_start",
	"placeholder_variables",
	"transition_grammar_dir",
	"transition_grammar_file",
]
# FIXME: Upload the rdf data at the startup via cold start.
class RdfStoreNode(DataDictionaryNode[_Parameters]):
	CoordsListParser = ListReferenceParser[_Parameters, CoordsList, Any]
	OffIntervalsParser = ListReferenceParser[_Parameters, list[TimeInterval], Any]

	def __init__(self, **kwArgs) -> None:
		parsers: dict[_Parameters, ParserBase[_Parameters, Any, Any]] = {
			"fuseki_server": StrParser[_Parameters](self, "fuseki_server"),
			"rdf_dir": StrParser[_Parameters](self, "rdf_dir"),
			"rdf_store": StrParser[_Parameters](self, "rdf_store"),
			"sparql_dir": StrParser[_Parameters](self, "sparql_dir"),
			"sparql_query_template": StrParser[_Parameters](self, "sparql_query_template"),
			"sparql_var_selectors": ListJsonParser[_Parameters, tuple[str, str], list[str], Any](self, "sparql_var_selectors"),
			"placeholder_bind": StrParser[_Parameters](self, "placeholder_bind"),
			"placeholder_order_by": StrParser[_Parameters](self, "placeholder_order_by"),
			"placeholder_selector_end": StrParser[_Parameters](self, "placeholder_selector_end"),
			"placeholder_selector_start": StrParser[_Parameters](self, "placeholder_selector_start"),
			"placeholder_variables": StrParser[_Parameters](self, "placeholder_variables"),
			"transition_grammar_dir": StrParser[_Parameters](self, "transition_grammar_dir"),
			"transition_grammar_file": StrParser[_Parameters](self, "transition_grammar_file"),
		}
		newKw = { "node_name": "dd_rdf", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(parsers, **newKw)
		self.__varToSparql = PredicateToQueryStr(
			get_package_share_directory(package_name),
			self["transition_grammar_dir"][0],
			self["transition_grammar_file"][0],
			self["sparql_dir"][0],
			self["sparql_var_selectors"],
		)
		self.__httpInterface = FusekiInterface(self, self["fuseki_server"][0], self["rdf_store"][0])
		RtBiInterfaces.createSpaceTimeService(self, self.__onSpaceTimeRequest)

	def __joinList(self, l: list[str], separator: str) -> str:
		# Remove duplicates
		l = list(dict.fromkeys(l))
		return separator.join(l)

	def __varsToFilter(self, variables: list[str]) -> str:
		variables = list(filter(lambda s: s.startswith("?p_"), variables))
		condition = self.__joinList(variables, " || ")
		return f"FILTER ({condition})"

	def __onSpaceTimeRequest(self, req: SpaceTime.Request, res: SpaceTime.Response) -> SpaceTime.Response:
		payload = ColdStartPayload(req.json_payload)
		sparqls: list[str] = [] # CSpell: ignore - sparqls
		variables: list[str] = []
		binds: list[str] = []
		# Polygon SPARQL must come first, order in optional statements matter. https://stackoverflow.com/a/61395608/750567
		variables.append(self.__varToSparql.polygonVarNames)
		sparqls.append(self.__varToSparql.selector("polygons", self["placeholder_selector_start"][0], self["placeholder_selector_end"][0]))
		i = 0
		for predicate in payload.predicates:
			(variable, sparql, bindStatement) = self.__varToSparql.transform(predicate, i, self["placeholder_selector_start"][0], self["placeholder_selector_end"][0])
			if variable == "" and sparql == "": continue
			variables.append(variable)
			sparqls.append(sparql)
			binds.append(bindStatement)
			i += 1
		binds.append(self.__varsToFilter(variables))

		query = Path(get_package_share_directory(package_name), self["sparql_dir"][0], self["sparql_query_template"][0]).read_text()
		query = query.replace(self["placeholder_variables"][0], self.__joinList(variables, " "))
		query = query.replace(self["placeholder_selector_end"][0], self.__joinList(sparqls, ""))
		query = query.replace(self["placeholder_bind"][0], self.__joinList(binds, "\n\t"))
		query = query.replace(self["placeholder_order_by"][0], self.__varToSparql.polygonVarNames)
		return self.__httpInterface.staticReachability(query, res)

	def render(self) -> None:
		return super().render()

def main(args=None):
	rclpy.init(args=args)
	ddNode = RdfStoreNode()
	rclpy.spin(ddNode)
	ddNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
