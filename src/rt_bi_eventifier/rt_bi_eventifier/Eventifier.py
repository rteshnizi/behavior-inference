from json import loads
from typing import cast

import networkx as nx
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.logging import LoggingSeverity
from rclpy.node import Publisher
from rclpy.parameter import Parameter

from rt_bi_commons.Base.ColdStartableNode import ColdStartable, ColdStartPayload
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.NetworkX import NxUtils
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber
from rt_bi_core.Spatial import MapPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_eventifier import package_name
from rt_bi_eventifier.Model.CrappyBA import CrappyBA
from rt_bi_eventifier.Model.MetricIGraph import MetricIGraph


class Eventifier(ColdStartable, RegionsSubscriber):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "eventifier", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		RegionsSubscriber.__init__(self, **newKw)
		ColdStartable.__init__(self)
		self.declareParameters()
		self.__renderModules: list[MetricIGraph.SUBMODULE] = []
		self.__grammarDir: str = ""
		self.__grammarFile: str = ""
		self.__states: list[str] = []
		self.__transitions: dict[str, dict[str, str]] = {}
		self.__start: str = ""
		self.__accepting: list[str] = []
		self.__baseDir = get_package_share_directory(package_name)
		self.parseParameters()
		self.__ba = CrappyBA(
			"BA",
			self.__states,
			self.__transitions,
			self.__start,
			self.__accepting,
			self.__baseDir,
			self.__grammarDir,
			self.__grammarFile
		)
		modulePublishers: dict[MetricIGraph.SUBMODULE, Publisher | None] = {}
		for module in MetricIGraph.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher

		self.__iGraphPublisher = RtBiInterfaces.createIGraphPublisher(self)
		self.__iGraph: MetricIGraph = MetricIGraph(modulePublishers)
		RtBiInterfaces.subscribeToProjectiveMap(self, self.enqueueUpdate)
		RtBiInterfaces.subscribeToPredicates(self, self.__onPredicates)
		self.waitForColdStartPermission()
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		RtBiInterfaces.subscribeToAffineMap(self, self.enqueueUpdate)
		RtBiInterfaces.subscribeToSensors(self, self.enqueueUpdate)
		self.publishColdStartDone()
		self.__ba.initFlask(self)
		return

	def __onPredicates(self, predicateJsonStr: str) -> None:
		symMap = loads(predicateJsonStr)
		self.__ba.setSymbolicNameOfPredicate(symMap)
		return

	def __publishBaEvent(self, iGraph: MetricIGraph, isomorphic: bool = False) -> None:
		# filterFn = lambda n: cast(NxUtils.Id, n).hIndex > (iGraph.hIndex - 1) - 2
		# g = cast(BehaviorIGraph, nx.subgraph_view(iGraph, filter_node=filterFn))
		# Ros.Log("SUB", g)
		if not self.__ba.initializedTokens: self.__ba.resetTokens(iGraph.history[-1].nodes)
		else: self.__ba.evaluate(iGraph)

		# if isomorphic: pass # Tell BA to update their token names
		# msg = Msgs.RtBi.IGraph()
		# msg.adjacency_json = iGraph.asStr()
		# self.__iGraphPublisher.publish(msg)
		return

	def __onUpdate(self, polygon: MapPolygon | SensingPolygon) -> None:
		self.log(f"Update received for polygon of type {polygon.type.name}.")
		self.__iGraph.updatePolygon(polygon, self.__publishBaEvent)
		if self.shouldRender:
			self.__iGraph.renderLatestCGraph()
			self.render()
		return

	def onMapUpdated(self, polygon: MapPolygon) -> None:
		return self.__onUpdate(polygon)

	def onSensorUpdated(self, polygon: SensingPolygon) -> None:
		return self.__onUpdate(polygon)

	def onTargetUpdated(self, _) -> None:
		# Eventifier receives the target observations through sensor emulator
		return

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("renderModules", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("grammar_dir", Parameter.Type.STRING)
		self.declare_parameter("grammar_file", Parameter.Type.STRING)
		self.declare_parameter("states", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_from", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_predicate", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("transitions_to", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("start", Parameter.Type.STRING)
		self.declare_parameter("accepting", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in MetricIGraph.SUBMODULES:
				self.__renderModules.append(module)
			else:
				self.log(f"Unknown module name in config file {module} for node {self.get_fully_qualified_name()}")
		self.__grammarDir = self.get_parameter("grammar_dir").get_parameter_value().string_value
		self.__grammarFile = self.get_parameter("grammar_file").get_parameter_value().string_value
		self.__states = list(self.get_parameter("states").get_parameter_value().string_array_value)
		frmList: list[str] = list(self.get_parameter("transitions_from").get_parameter_value().string_array_value)
		prdList: list[str] = list(self.get_parameter("transitions_predicate").get_parameter_value().string_array_value)
		toList: list[str] = list(self.get_parameter("transitions_to").get_parameter_value().string_array_value)
		for i in range(len(frmList)):
			frmState = frmList[i]
			toState = toList[i]
			prd = prdList[i]
			if frmState not in self.__transitions: self.__transitions[frmState] = {}
			self.__transitions[frmState][toState] = prd

		self.__start = self.get_parameter("start").get_parameter_value().string_value
		self.__accepting = list(self.get_parameter("accepting").get_parameter_value().string_array_value)
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		return markers

def main(args=None) -> None:
	rclpy.init(args=args)
	node = Eventifier()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
