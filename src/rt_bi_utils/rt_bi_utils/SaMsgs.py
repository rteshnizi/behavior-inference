from collections import UserList
from math import nan
from typing import AbstractSet, Callable, Sequence, Tuple, Union

from rclpy.node import Client, Node, Publisher, Service, Timer
from sa_msgs.msg import EstimationMsg, FeatureInfo, Fov as SaFovMsg, Pose as SaPoseMsg, PoseArray as SaPoseArray, RobotState
from sa_msgs.srv import QueryFeature

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Pose import Pose


class SaMsgs:
	"""
		This class only prepares the messages defined in the TAMU AGC SA project.
	"""
	__MAP_UPDATE_TOPIC = "/sa_map/FeatureMap_BIL"
	__MAP_SERVICE_NAME = "/sa_map/feature_query"
	__ESTIMATION_TOPIC = "/sa_tajectory_estimation/estimation" # CSpell: disable-line -- typo is from Tianqi ＼（〇_ｏ）／
	__ROBOT_STATE_TOPIC = "/sa_world_model/true_objects_state"

	@staticmethod
	def convertSaPoseToPose(saPose: SaPoseMsg, timeNanoSecs: int = 0) -> Pose:
		return Pose(timeNanoSecs, saPose.x, saPose.y, saPose.yaw)

	@staticmethod
	def convertSaPoseToCoords(saPose: SaPoseMsg) -> Geometry.Coords:
		return (saPose.x, saPose.y)

	@staticmethod
	def convertSaPoseListToCoordsList(saPoseSeq: Union[Sequence[SaPoseMsg], AbstractSet[SaPoseMsg], UserList[SaPoseMsg]]) -> Geometry.CoordsList:
		coords: Geometry.CoordsList = []
		for saPose in saPoseSeq:
			RosUtils.AppendMessage(coords, SaMsgs.convertSaPoseToCoords(saPose))
		return coords

	@staticmethod
	def createSaPoseMsg(x: float, y: float, yaw = 0.0) -> SaPoseMsg:
		"""
		Create a Point Msg.

		Parameters
		----------
		x : float
		y : float
		z : float, optional
			by default 0.0
		Returns
		-------
		PointMsg
		"""
		message = SaPoseMsg()
		message.x = float(x)
		message.y = float(y)
		message.yaw = float(yaw)
		return message

	@staticmethod
	def createSaPoseArrayMsg(coords: Geometry.CoordsList) -> SaPoseArray:
		"""
		Create a PoseArray Message.

		Parameters
		----------
		coords : Geometry.CoordsList
			A list of (x, y), tuples

		Returns
		-------
		PoseArray
			The message.
		"""
		message = SaPoseArray()
		for coord in coords:
			RosUtils.AppendMessage(message.traj, SaMsgs.createSaPoseMsg(*coord))
		return message

	@staticmethod
	def createSaFovMsg(coords: Geometry.CoordsList) -> SaFovMsg:
		"""
		Create a Fov Message. It's literally the same as the PoseArray ＼(ﾟｰﾟ＼) ( ﾉ ﾟｰﾟ)ﾉ

		Parameters
		----------
		coords : Geometry.CoordsList
			A list of (x, y), tuples

		Returns
		-------
		Fov
			The message.
		"""
		message = SaFovMsg()
		for coord in coords:
			RosUtils.AppendMessage(message.corners, SaMsgs.createSaPoseMsg(*coord))
		return message

	@staticmethod
	def subscribeToMapUpdateTopic(node: Node, callbackFunc: Callable[[FeatureInfo], None]) -> None:
		RosUtils.CreateSubscriber(node, FeatureInfo, SaMsgs.__MAP_UPDATE_TOPIC, callbackFunc) # type: ignore - "type[Metaclass_FeatureInfo]" is incompatible with "type[FeatureInfo]"

	@staticmethod
	def createSaRobotStatePublisher(node: Node, callbackFunc: Callable = lambda: None, intervalSecs: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, RobotState, SaMsgs.__ROBOT_STATE_TOPIC, callbackFunc, intervalSecs)

	@staticmethod
	def subscribeToSaRobotStateTopic(node: Node, callbackFunc: Callable[[RobotState], None]) -> None:
		RosUtils.CreateSubscriber(node, RobotState, SaMsgs.__ROBOT_STATE_TOPIC, callbackFunc) # type: ignore - "type[Metaclass_RobotState]" is incompatible with "type[RobotState]"

	@staticmethod
	def subscribeToSaEstimationTopic(node: Node, callbackFunc: Callable[[EstimationMsg], None]) -> None:
		RosUtils.CreateSubscriber(node, EstimationMsg, SaMsgs.__ESTIMATION_TOPIC, callbackFunc) # type: ignore - "type[Metaclass_EstimationMsg]" is incompatible with "type[EstimationMsg]"

	@staticmethod
	def createSaFeatureQueryService(node: Node, callbackFunc: Callable[[QueryFeature.Request, QueryFeature.Response], QueryFeature.Response]) -> Service:
		return node.create_service(QueryFeature, SaMsgs.__MAP_SERVICE_NAME, callbackFunc)

	@staticmethod
	def createSaFeatureQueryClient(node: Node) -> Client:
		return node.create_client(QueryFeature, SaMsgs.__MAP_SERVICE_NAME)
