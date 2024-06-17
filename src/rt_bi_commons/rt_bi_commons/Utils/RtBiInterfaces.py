from enum import Enum
from math import nan

from rclpy.node import Client, Publisher, Service, Timer
from typing_extensions import Callable, NamedTuple, deprecated

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs


class ReferenceDescriptor(NamedTuple):
	node: str
	param: str

	def __repr__(self) -> str:
		return str(self)

	def __str__(self) -> str:
		return RtBiNode.toServiceName(self.node, self.param)

	@property
	def serviceName(self) -> str:
		return RtBiNode.toServiceName(self.node, self.param)

	@staticmethod
	def fromRefStr(ref: str) -> "ReferenceDescriptor":
		parts = [s.strip("_") for s in ref.split("/")]
		return ReferenceDescriptor(node=parts[1].lower(), param=parts[2].lower())

class RtBiInterfaces:
	BA_NODE_PREFIX = "/rt_bi_behavior/ba"
	KNOWN_REGION_NODE_PREFIX = "/rt_bi_emulator/kn"
	class TopicNames(Enum):
		RT_BI_EMULATOR_KNOWN = "/__rt_bi_emulator/known"
		RT_BI_EMULATOR_MAP = "/__rt_bi_runtime/map"
		RT_BI_EMULATOR_SENSOR = "/__rt_bi_emulator/sensor"
		RT_BI_EMULATOR_TARGET = "/__rt_bi_emulator/target"
		RT_BI_EVENTIFIER_IGRAPH = "/__rt_bi_eventifier/b_igraph"
		RT_BI_EVENTIFIER_ISOMORPHISM = "/__rt_bi_eventifier/isomorphism"
		RT_BI_RUNTIME_COLD_START = "/__rt_bi_runtime/cold_start"
		RT_BI_RUNTIME_PREDICATES = "/__rt_bi_runtime/predicates"

	class ServiceNames(Enum):
		RT_BI_EVENTIFIER_EVALUATE = "/__rt_bi_eventifier/eval"
		RT_BI_RUNTIME_DD_RDF = "/__rt_bi_runtime/dd_rdf"

	@staticmethod
	def createSpacePublisher(node: RtBiNode, topic: TopicNames, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return Ros.CreatePublisher(node, Msgs.RtBi.RegularSetArray, topic.value, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSpace(node: RtBiNode, topic: TopicNames, callbackFunc: Callable[[Msgs.RtBi.RegularSetArray], None]) -> None:
		Ros.CreateSubscriber(node, Msgs.RtBi.RegularSetArray, topic.value, callbackFunc)
		return

	@staticmethod
	def createSensorPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return RtBiInterfaces.createSpacePublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SENSOR, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSensors(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.RegularSetArray], None]) -> None:
		RtBiInterfaces.subscribeToSpace(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SENSOR, callbackFunc)
		return

	@staticmethod
	def createKnownRegionPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return RtBiInterfaces.createSpacePublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_KNOWN, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToAffineMap(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.RegularSetArray], None]) -> None:
		RtBiInterfaces.subscribeToSpace(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_KNOWN, callbackFunc)
		return

	@staticmethod
	def createTargetPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return RtBiInterfaces.createSpacePublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_TARGET, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToTargets(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.RegularSetArray], None]) -> None:
		RtBiInterfaces.subscribeToSpace(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_TARGET, callbackFunc)
		return

	@staticmethod
	def createProjectiveMapPublisher(node: RtBiNode) -> Publisher:
		(publisher, _) = RtBiInterfaces.createSpacePublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_MAP)
		return publisher

	@staticmethod
	def subscribeToProjectiveMap(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.RegularSetArray], None]) -> None:
		RtBiInterfaces.subscribeToSpace(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_MAP, callbackFunc)
		return

	@staticmethod
	def createPredicatesPublisher(node: RtBiNode) -> Publisher:
		(publisher, _) = Ros.CreatePublisher(node, Msgs.Std.String, RtBiInterfaces.TopicNames.RT_BI_RUNTIME_PREDICATES.value)
		return publisher

	@staticmethod
	def subscribeToPredicates(node: RtBiNode, callbackFunc: Callable[[str], None]) -> None:
		Ros.CreateSubscriber(node, Msgs.Std.String, RtBiInterfaces.TopicNames.RT_BI_RUNTIME_PREDICATES.value, lambda m: callbackFunc(m.data))
		return

	@deprecated("Data ref nodes are replaced by RDF", category=None)
	@staticmethod
	def createDataReferenceService(node: RtBiNode, paramName: str, callbackFunc: Callable[[Msgs.RtBiSrv.DataReference.Request, Msgs.RtBiSrv.DataReference.Response], Msgs.RtBiSrv.DataReference.Response]) -> Service:
		ddServiceName = RtBiNode.toServiceName(node.get_name(), paramName)
		return Ros.CreateService(node, Msgs.RtBiSrv.DataReference, ddServiceName, callbackFunc)

	@deprecated("Data ref nodes are replaced by RDF", category=None)
	@staticmethod
	def createDataReferenceClient(node: RtBiNode, ref: ReferenceDescriptor) -> Client:
		return Ros.CreateClient(node, Msgs.RtBiSrv.DataReference, ref.serviceName)

	@staticmethod
	def createSpaceTimeService(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBiSrv.SpaceTime.Request, Msgs.RtBiSrv.SpaceTime.Response], Msgs.RtBiSrv.SpaceTime.Response]) -> Service:
		svcName = f"{RtBiInterfaces.ServiceNames.RT_BI_RUNTIME_DD_RDF.value}/space_time"
		svc = Ros.CreateService(node, Msgs.RtBiSrv.SpaceTime, svcName, callbackFunc)
		return svc

	@staticmethod
	def createSpaceTimeClient(node: RtBiNode) -> Client:
		svcName = f"{RtBiInterfaces.ServiceNames.RT_BI_RUNTIME_DD_RDF.value}/space_time"
		return Ros.CreateClient(node, Msgs.RtBiSrv.SpaceTime, svcName)

	@staticmethod
	def createColdStartPublisher(node: RtBiNode) -> Publisher:
		(publisher, _) = Ros.CreatePublisher(node, Msgs.RtBi.ColdStart, RtBiInterfaces.TopicNames.RT_BI_RUNTIME_COLD_START.value)
		return publisher

	@staticmethod
	def subscribeToColdStart(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.ColdStart], None]) -> None:
		Ros.CreateSubscriber(node, Msgs.RtBi.ColdStart, RtBiInterfaces.TopicNames.RT_BI_RUNTIME_COLD_START.value, callbackFunc)
		return

	@staticmethod
	def createIGraphPublisher(node: RtBiNode) -> Publisher:
		(publisher, _) = Ros.CreatePublisher(node, Msgs.RtBi.IGraph, RtBiInterfaces.TopicNames.RT_BI_EVENTIFIER_IGRAPH.value)
		return publisher

	@staticmethod
	def subscribeToIGraph(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.IGraph], None]) -> None:
		Ros.CreateSubscriber(node, Msgs.RtBi.IGraph, RtBiInterfaces.TopicNames.RT_BI_EVENTIFIER_IGRAPH.value, callbackFunc)
		return

	@staticmethod
	def createIsomorphismPublisher(node: RtBiNode) -> Publisher:
		(publisher, _) = Ros.CreatePublisher(node, Msgs.RtBi.Isomorphism, RtBiInterfaces.TopicNames.RT_BI_EVENTIFIER_ISOMORPHISM.value)
		return publisher

	@staticmethod
	def subscribeToIsomorphism(node: RtBiNode, callbackFunc: Callable[[Msgs.RtBi.Isomorphism], None]) -> None:
		Ros.CreateSubscriber(node, Msgs.RtBi.Isomorphism, RtBiInterfaces.TopicNames.RT_BI_EVENTIFIER_ISOMORPHISM.value, callbackFunc)
		return
