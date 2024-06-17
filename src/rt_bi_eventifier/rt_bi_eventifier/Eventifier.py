from json import dumps

from rclpy.parameter import Parameter

from rt_bi_commons.Base.ColdStartableNode import ColdStartable, ColdStartPayload
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.RegionsSubscriber import RegionsSubscriber
from rt_bi_core.Spatial import MapPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_eventifier.Model.MetricIGraph import MetricIGraph


class Eventifier(ColdStartable, RegionsSubscriber):
	def __init__(self, **kwArgs) -> None:
		newKw = { "node_name": "eventifier", "loggingSeverity": Ros.LoggingSeverity.INFO, **kwArgs}
		RegionsSubscriber.__init__(self, **newKw)
		ColdStartable.__init__(self)
		self.declareParameters()
		self.__renderModules: list[MetricIGraph.SUBMODULE] = []
		self.parseParameters()
		modulePublishers: dict[MetricIGraph.SUBMODULE, Ros.Publisher | None] = {}
		for module in MetricIGraph.SUBMODULES:
			if module in self.__renderModules: (publisher, _) = RViz.createRVizPublisher(self, Ros.CreateTopicName(module))
			else: publisher = None
			modulePublishers[module] = publisher

		self.__iGraphPublisher = RtBiInterfaces.createIGraphPublisher(self)
		self.__isoPublisher = RtBiInterfaces.createIsomorphismPublisher(self)
		self.__iGraph: MetricIGraph = MetricIGraph(modulePublishers)
		RtBiInterfaces.subscribeToProjectiveMap(self, self.enqueueUpdate)
		self.waitForColdStartPermission()
		return

	def onColdStartAllowed(self, payload: ColdStartPayload) -> None:
		RtBiInterfaces.subscribeToAffineMap(self, self.enqueueUpdate)
		RtBiInterfaces.subscribeToSensors(self, self.enqueueUpdate)
		self.publishColdStartDone()
		return

	def __publishBaMsg(self, iGraph: MetricIGraph, isomorphism: dict[str, str]) -> None:
		if len(isomorphism) > 0:
			msg = Msgs.RtBi.Isomorphism()
			msg.isomorphism_json = dumps(isomorphism)
			Ros.Publish(self.__isoPublisher, msg)
		else:
			msg = Msgs.RtBi.IGraph()
			msg.adjacency_json = iGraph.asStr()
			Ros.Publish(self.__iGraphPublisher, msg)
		return

	def __onUpdate(self, polygon: MapPolygon | SensingPolygon) -> None:
		self.log(f"Update received for polygon of type {polygon.type.name}.")
		self.__iGraph.updatePolygon(polygon, self.__publishBaMsg)
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
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		yamlModules = self.get_parameter("renderModules").get_parameter_value().string_array_value
		for module in yamlModules:
			if module in MetricIGraph.SUBMODULES:
				self.__renderModules.append(module)
			else:
				self.log(f"Unknown module name in config file {module} for node {self.get_fully_qualified_name()}")
		return

	def createMarkers(self) -> list[RViz.Msgs.Marker]:
		markers = []
		return markers

def main(args=None) -> None:
	return Eventifier.Main(args)

if __name__ == "__main__":
	main()
