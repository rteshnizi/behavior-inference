from enum import Enum
from math import nan
from typing import Callable, Sequence, Tuple, Union

from geometry_msgs.msg import Point as PointMsg, Point32 as Point32Msg, Polygon as PolygonMsg, Pose as PoseMsg, Quaternion as QuaternionMsg
from rclpy.node import Publisher, Timer

import rt_bi_utils.Ros as RosUtils
from rt_bi_interfaces.msg import DynamicRegion, EstimationMsg
from rt_bi_utils.Geometry import Geometry, Polygon
from rt_bi_utils.Pose import Coords, CoordsList, Pose
from rt_bi_utils.RtBiNode import RtBiNode


class RtBiInterfaces:
	class TopicNames(Enum):
		RT_BI_EMULATOR_SENSOR = "/rt_bi_emulator/sensor"
		RT_BI_EMULATOR_SYMBOL = "/rt_bi_emulator/symbol"
		RT_BI_EMULATOR_TARGET = "/rt_bi_emulator/target"
		RT_BI_EMULATOR_MAP = "/rt_bi_emulator/map"
		RT_BI_EMULATOR_ESTIMATION = "/rt_bi_emulator/estimation"

	@staticmethod
	def createRegionPublisher(node: RtBiNode, topic: TopicNames, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, DynamicRegion, topic.value, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToRegionTopic(node: RtBiNode, topic: TopicNames, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RosUtils.CreateSubscriber(node, DynamicRegion, topic.value, callbackFunc) # type: ignore - "type[Metaclass_DynamicRegion]" is incompatible with "type[DynamicRegion]"

	@staticmethod
	def createSensorPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RtBiInterfaces.createRegionPublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SENSOR, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSensorTopic(node: RtBiNode, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RtBiInterfaces.subscribeToRegionTopic(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SENSOR, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def createSymbolPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RtBiInterfaces.createRegionPublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SYMBOL, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSymbolTopic(node: RtBiNode, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RtBiInterfaces.subscribeToRegionTopic(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_SYMBOL, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def createTargetPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RtBiInterfaces.createRegionPublisher(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_TARGET, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToTargetTopic(node: RtBiNode, callbackFunc: Callable[[DynamicRegion], None]) -> None:
		RtBiInterfaces.subscribeToRegionTopic(node, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_TARGET, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def fromStdPointToCoords(p: PointMsg) -> Coords:
		return (p.x, p.y)

	@staticmethod
	def fromStdPoints32ToCoordsList(points: Sequence[Point32Msg]) -> CoordsList:
		return [(p.x, p.y) for p in points]

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
	def createEstimationPublisher(node: RtBiNode, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, EstimationMsg, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_ESTIMATION.value, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToEstimationTopic(node: RtBiNode, callbackFunc: Callable[[EstimationMsg], None]) -> None:
		RosUtils.CreateSubscriber(node, EstimationMsg, RtBiInterfaces.TopicNames.RT_BI_EMULATOR_ESTIMATION.value, callbackFunc) # type: ignore - "type[Metaclass_EstimationMsg]" is incompatible with "type[EstimationMsg]"

	# @staticmethod
	# def createSaFeatureQueryService(node: RtBiNode, callbackFunc: Callable[[QueryFeature.Request, QueryFeature.Response], QueryFeature.Response]) -> Service:
	# 	return node.create_service(QueryFeature, SaMsgs.__MAP_SERVICE_NAME, callbackFunc)

	# @staticmethod
	# def createSaFeatureQueryClient(node: RtBiNode) -> Client:
	# 	return node.create_client(QueryFeature, SaMsgs.__MAP_SERVICE_NAME)
