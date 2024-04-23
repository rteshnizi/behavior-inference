from json import dumps
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
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.srv import SpaceTime
from rt_bi_runtime import package_name
from rt_bi_runtime.Model.FusekiInterface import FusekiInterface
from rt_bi_runtime.Model.SparqlTransformer import PredicateToQueryStr

_Parameters = Literal[
	"fuseki_server",
	"rdf_dir",
	"rdf_store",
	"sparql_dir",
	"sparql_template_sets",
	"sparql_template_ids",
	"sparql_var_selectors",
	"placeholder_bind",
	"placeholder_ids",
	"placeholder_order",
	"placeholder_selector",
	"placeholder_variables",
	"transition_grammar_dir",
	"transition_grammar_file",
]
# FIXME: Upload the rdf data at the startup via cold start.
class RdfStoreNode(DataDictionaryNode[_Parameters]):
	def __init__(self, **kwArgs) -> None:
		parsers: dict[_Parameters, ParserBase[_Parameters, Any, Any]] = {
			"fuseki_server": StrParser[_Parameters](self, "fuseki_server"),
			"rdf_dir": StrParser[_Parameters](self, "rdf_dir"),
			"rdf_store": StrParser[_Parameters](self, "rdf_store"),
			"sparql_dir": StrParser[_Parameters](self, "sparql_dir"),
			"sparql_template_sets": StrParser[_Parameters](self, "sparql_template_sets"),
			"sparql_template_ids": StrParser[_Parameters](self, "sparql_template_ids"),
			"sparql_var_selectors": ListJsonParser[_Parameters, tuple[str, str], list[str], Any](self, "sparql_var_selectors"),
			"placeholder_bind": StrParser[_Parameters](self, "placeholder_bind"),
			"placeholder_ids": StrParser[_Parameters](self, "placeholder_ids"),
			"placeholder_order": StrParser[_Parameters](self, "placeholder_order"),
			"placeholder_selector": StrParser[_Parameters](self, "placeholder_selector"),
			"placeholder_variables": StrParser[_Parameters](self, "placeholder_variables"),
			"transition_grammar_dir": StrParser[_Parameters](self, "transition_grammar_dir"),
			"transition_grammar_file": StrParser[_Parameters](self, "transition_grammar_file"),
		}
		newKw = { "node_name": "dd_rdf", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(parsers, **newKw)
		self.__sparqlXfmr = PredicateToQueryStr(
			get_package_share_directory(package_name),
			self["transition_grammar_dir"][0],
			self["transition_grammar_file"][0],
			self["sparql_dir"][0],
			self["placeholder_order"][0],
			self["placeholder_selector"][0],
			self["placeholder_variables"][0],
			self["sparql_var_selectors"],
		)
		self.__httpInterface = FusekiInterface(self, self["fuseki_server"][0], self["rdf_store"][0])
		RtBiInterfaces.createSpaceTimeService(self, self.__onSpaceTimeRequest)

	def __joinList(self, l: list[str], separator: str) -> str:
		l = list(dict.fromkeys(l)) # Remove duplicates
		return separator.join(l)

	def __createFilterStatement(self, variables: list[str]) -> str:
		variables = list(filter(lambda s: s.startswith("?p_"), variables))
		condition = self.__joinList(variables, " || ")
		return f"FILTER ({condition})"

	def __fillTemplate(self, templateName: str, namespace: str, ids: list[str], whereClauses: list[str], variables: list[str], binds: list[str], orders: list[str]) -> str:
		sparql = Path(
			get_package_share_directory(package_name),
			self["sparql_dir"][0],
			templateName,
		).read_text()
		sparql = sparql.replace("?namespace", namespace)
		sparql = sparql.replace(self["placeholder_variables"][0], self.__joinList(variables, "\n\t"))
		sparql = sparql.replace(self["placeholder_ids"][0], self.__joinList(ids, "\n\t\t"))
		sparql = sparql.replace(self["placeholder_selector"][0], self.__joinList(whereClauses, ""))
		sparql = sparql.replace(self["placeholder_bind"][0], self.__joinList(binds, "\n\t"))
		sparql = sparql.replace(self["placeholder_order"][0], self.__joinList(orders, " "))
		return sparql

	def __onSpaceTimeRequest(self, req: SpaceTime.Request, res: SpaceTime.Response) -> SpaceTime.Response:
		payload = ColdStartPayload(req.json_payload)
		if payload.nodeName == RtBiInterfaces.DYNAMIC_MAP_NODE_NAME:
			if not payload.done: res = self.__fetchStaticReachability(payload, res)
			else: res = self.__fetchDynamicReachability(payload, res)
		else:
			self.log(f"Unexpected request payload in {self.__class__.__name__}: \n\t{req.json_payload}")
		return res

	def __fetchStaticReachability(self, payload: ColdStartPayload, res: SpaceTime.Response) -> SpaceTime.Response:
		predicateMapping: dict[str, str] = {}
		whereClauses: list[str] = []
		variables: list[str] = []
		binds: list[str] = []
		orders: list[str] = []
		(extractedSelector, extractedVars, extractedOrders) = self.__sparqlXfmr.selector("polygons")
		whereClauses.append(extractedSelector)
		variables.append(extractedVars)
		orders.append(extractedOrders)
		i = 0
		for predicate in payload.predicates:
			(extractedSelector, extractedVars, extractedBindings, extractedOrders) = self.__sparqlXfmr.transformPredicate(predicate, i)
			if extractedVars == "" and extractedSelector == "": continue
			predicateMapping[predicate] = extractedVars
			variables.append(extractedVars)
			whereClauses.append(extractedSelector)
			binds.append(extractedBindings)
			orders.append(extractedOrders)
			i += 1
		binds.append(self.__createFilterStatement(variables))

		sparql = self.__fillTemplate(
			self["sparql_template_sets"][0],
			"rt_bi:RegularSpace",
			[""],
			whereClauses,
			variables,
			binds,
			orders
		)
		res.json_predicate_symbols = dumps(predicateMapping)
		return self.__httpInterface.staticReachability(sparql, res)

	def __fetchDynamicReachability(self, payload: ColdStartPayload, res: SpaceTime.Response) -> SpaceTime.Response:
		whereClauses: list[str] = []
		variables: list[str] = []
		binds: list[str] = []
		orders: list[str] = []
		(extractedSelector, extractedVars, extractedOrders) = self.__sparqlXfmr.selector("accessible-times")
		whereClauses.append(extractedSelector)
		variables.append(extractedVars)
		orders.append(extractedOrders)
		sparql = self.__fillTemplate(
			self["sparql_template_ids"][0],
			"rt_bi:RegularSpace",
			[f"<{iri}>" for iri in payload.dynamic],
			whereClauses,
			variables,
			binds,
			orders
		)
		return self.__httpInterface.dynamicReachability(sparql, res)

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
