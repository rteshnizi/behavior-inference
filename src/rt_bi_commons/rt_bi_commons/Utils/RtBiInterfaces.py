from enum import Enum
from math import nan
from typing import AbstractSet, Callable, NamedTuple, Sequence

from geometry_msgs.msg import Point as PointMsg, Point32 as Point32Msg, Polygon as PolygonMsg, Pose as PoseMsg, Quaternion as QuaternionMsg
from rclpy.node import Client, Publisher, Service, Timer

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Pose import Coords, CoordsList, Pose
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import Geometry, Polygon
from rt_bi_interfaces.msg import DynamicRegion, EstimationMsg
from rt_bi_interfaces.srv import DataReference, SpaceTime


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
	class TopicNames(Enum):
		RT_BI_EMULATOR_MAP = "/__rt_bi_emulator/em_dynamic_map"
		RT_BI_EMULATOR_SENSOR = "/__rt_bi_emulator/sensor"
		RT_BI_EMULATOR_SYMBOL = "/__rt_bi_emulator/symbol"
		RT_BI_EMULATOR_TARGET = "/__rt_bi_emulator/target"
		RT_BI_EMULATOR_ESTIMATION = "/__rt_bi_emulator/estimation"
		RT_BI_RUNTIME_MAP_REGIONS = "/__rt_bi_runtime/map_regions"

	class ServiceNames(Enum):
		RT_BI_RUNTIME_DD_RDF = "/__rt_bi_runtime/dd_rdf"

	@staticmethod
	def createRegionPublisher(node: RtBiNode, topic: TopicNames, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return Ros.CreatePublisher(node, DynamicRegion, topic.value, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToRegion(node: RtBiNode, topic: TopicNames, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		Ros.CreateSubscriber(node, DynamicRegion, topic.value, callbackFunc) # type: ignore - "type[Metaclass_DynamicRegion]" is incompatible with "type[DynamicRegion]"

	@staticmethod
	def createSensorPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return RtBiInterfaces.createRegionPublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SENSOR, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSensor(node: RtBiNode, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RtBiInterfaces.subscribeToRegion(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SENSOR, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def createSymbolPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return RtBiInterfaces.createRegionPublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SYMBOL, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSymbol(node: RtBiNode, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RtBiInterfaces.subscribeToRegion(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SYMBOL, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def createTargetPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return RtBiInterfaces.createRegionPublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_TARGET, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToTarget(node: RtBiNode, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RtBiInterfaces.subscribeToRegion(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_TARGET, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def fromStdPointToCoords(p: PointMsg) -> Coords:
		return (p.x, p.y)

	@staticmethod
	def fromStdPoints32ToCoordsList(pts: Sequence[Point32Msg] | AbstractSet[Point32Msg] | list[Point32Msg]) -> CoordsList:
		return [(p.x, p.y) for p in pts]

	@staticmethod
	def toStdPoint(pose: Pose) -> PointMsg:
		return PointMsg(x=pose.x, y=pose.y, z=0.0)

	@staticmethod
	def toStdPose(pose: Pose) -> PoseMsg:
		pointMsg = PointMsg(x=pose.x, y=pose.y, z=0.0)
		quat = pose.angleAsQuaternion()
		quadMsg = QuaternionMsg(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
		return PoseMsg(position=pointMsg, orientation=quadMsg)

	@staticmethod
	def toPolygonMsg(poly: Polygon) -> PolygonMsg:
		msg = PolygonMsg(points=[Point32Msg(x=p[0], y=p[1], z=0.0) for p in Geometry.getGeometryCoords(poly)])
		return msg

	@staticmethod
	def createEstimationPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> tuple[Publisher, Timer | None]:
		return Ros.CreatePublisher(node, EstimationMsg, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_ESTIMATION.value, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToEstimation(node: RtBiNode, callbackFunc: Callable[[EstimationMsg], None]) -> None:
		Ros.CreateSubscriber(node, EstimationMsg, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_ESTIMATION.value, callbackFunc) # type: ignore - "type[Metaclass_EstimationMsg]" is incompatible with "type[EstimationMsg]"

	@staticmethod
	def createDataReferenceService(node: RtBiNode, paramName: str, callbackFunc: Callable[[DataReference.Request, DataReference.Response], DataReference.Response]) -> Service:
		ddServiceName = RtBiNode.toServiceName(node.get_name(), paramName)
		return Ros.CreateService(node, DataReference, ddServiceName, callbackFunc)

	@staticmethod
	def createDataReferenceClient(node: RtBiNode, ref: ReferenceDescriptor) -> Client:
		return Ros.CreateClient(node, DataReference, ref.serviceName)

	@staticmethod
	def createSpaceTimeService(node: RtBiNode, callbackFunc: Callable[[SpaceTime.Request, SpaceTime.Response], SpaceTime.Response]) -> Service:
		svcName = f"{RtBiInterfaces.ServiceNames.RT_BI_RUNTIME_DD_RDF.value}/space_time"
		svc = Ros.CreateService(node, SpaceTime, svcName, callbackFunc)
		return svc

	@staticmethod
	def createSpaceTimeClient(node: RtBiNode) -> Client:
		svcName = f"{RtBiInterfaces.ServiceNames.RT_BI_RUNTIME_DD_RDF.value}/space_time"
		return Ros.CreateClient(node, SpaceTime, svcName)

	@staticmethod
	def createMapRegionsPublisher(node: RtBiNode) -> Publisher:
		(publisher, _) = Ros.CreatePublisher(node, SpaceTime.Response, RtBiInterfaces.TopicNames.RT_BI_RUNTIME_MAP_REGIONS.value)
		return publisher

	@staticmethod
	def subscribeToMapRegions(node: RtBiNode, callbackFunc: Callable[[SpaceTime.Response], None]) -> None:
		Ros.CreateSubscriber(node, SpaceTime.Response, RtBiInterfaces.TopicNames.RT_BI_RUNTIME_MAP_REGIONS.value, callbackFunc) # type: ignore
		return
