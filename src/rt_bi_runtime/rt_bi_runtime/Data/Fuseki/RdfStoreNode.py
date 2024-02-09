from pathlib import Path
from typing import Any, Literal

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import LoggingSeverity

from rt_bi_commons.Base.DataDictionaryNode import DataDictionaryNode
from rt_bi_commons.Base.ParamParserBase import ParserBase
from rt_bi_commons.RosParamParsers.AtomicParsers import StrParser
from rt_bi_commons.RosParamParsers.ReferenceParser import ListReferenceParser
from rt_bi_commons.Shared.Pose import CoordsList
from rt_bi_commons.Shared.TimeInterval import TimeInterval
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_interfaces.srv import StaticReachability
from rt_bi_runtime import package_name
from rt_bi_runtime.Data.Fuseki.HttpInterface import HttpInterface
from rt_bi_runtime.Data.SparqlHelper import SparqlHelper

_K = Literal["fuseki_server", "rdf_store", "rdf_directory_name", "sparql_directory_name"]
class RdfStoreNode(DataDictionaryNode[_K]):
	CoordsListParser = ListReferenceParser[_K, CoordsList, Any]
	OffIntervalsParser = ListReferenceParser[_K, list[TimeInterval], Any]

	def __init__(self, **kwArgs) -> None:
		parsers: dict[_K, ParserBase[_K, Any, Any]] = {
			"fuseki_server": StrParser[_K](self, "fuseki_server"),
			"rdf_store": StrParser[_K](self, "rdf_store"),
			"rdf_directory_name": StrParser[_K](self, "rdf_directory_name"),
			"sparql_directory_name": StrParser[_K](self, "sparql_directory_name"),
		}
		newKw = { "node_name": "dd_rdf", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(parsers, **newKw)
		sparqlDir = str(Path(get_package_share_directory(package_name), self["sparql_directory_name"][0]).resolve())
		self.__httpInterface = HttpInterface(self["fuseki_server"][0], self["rdf_store"][0])
		self.__sparqlHelper = SparqlHelper(sparqlDir)
		RtBiInterfaces.createStaticReachabilityService(self, self.__onStaticReachability)

	def __onStaticReachability(self, req: StaticReachability.Request, res: StaticReachability.Response) -> StaticReachability.Response:
		queryStr = self.__sparqlHelper.query("reachability", req)
		return self.__httpInterface.staticReachability(queryStr, res)

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
