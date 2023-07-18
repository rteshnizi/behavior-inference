from math import nan
from typing import Callable, Tuple, Union, Sequence

from rclpy.node import Client, Node, Publisher, Service, Timer

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry
from sa_msgs.msg import EstimationMsg, FeatureInfo
from sa_msgs.msg import Fov as SaFovMsg
from sa_msgs.msg import Pose as SaPoseMsg
from sa_msgs.msg import PoseArray as SaPoseArray
from sa_msgs.msg import RobotState
from sa_msgs.srv import QueryFeature


class SaMsgs:
	"""
		This class only prepares the messages defined in the TAMU AGC SA project.
	"""
	__MAP_UPDATE_TOPIC = "/sa_map/FeatureMap_BIL"
	__MAP_SERVICE_NAME = "/sa_map/feature_query"
	__ESTIMATION_TOPIC = "/sa_tajectory_estimation/estimation" # typo is from Tianqi ＼（〇_ｏ）／
	__ROBOT_STATE_TOPIC = "/sa_world_model/true_objects_state"

	@staticmethod
	def convertSaPoseListToCoordsList(saPoseSeq: Sequence[SaPoseMsg]) -> Geometry.CoordsList:
		coords: Geometry.CoordsList = []
		for saPose in saPoseSeq:
			coords.append((saPose.x, saPose.y))
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
			message.traj.append(SaMsgs.createSaPoseMsg(*coord))
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
			message.corners.append(SaMsgs.createSaPoseMsg(*coord))
		return message

	@staticmethod
	def subscribeToMapUpdateTopic(node: Node, callbackFunc: Callable[[FeatureInfo], None]) -> None:
		RosUtils.CreateSubscriber(node, FeatureInfo, SaMsgs.__MAP_UPDATE_TOPIC, callbackFunc)

	@staticmethod
	def createSaRobotStatePublisher(node: Node, callbackFunc: Callable = lambda: None, interval: float = nan) -> Tuple[Publisher, Union[Timer, None]]:
		return RosUtils.CreatePublisher(node, RobotState, SaMsgs.__ROBOT_STATE_TOPIC, callbackFunc, interval)

	@staticmethod
	def subscribeToSaRobotStateTopic(node: Node, callbackFunc: Callable[[RobotState], None]) -> None:
		RosUtils.CreateSubscriber(node, RobotState, SaMsgs.__ROBOT_STATE_TOPIC, callbackFunc)

	@staticmethod
	def subscribeToSaEstimationTopic(node: Node, callbackFunc: Callable[[EstimationMsg], None]) -> None:
		RosUtils.CreateSubscriber(node, EstimationMsg, SaMsgs.__ESTIMATION_TOPIC, callbackFunc)

	@staticmethod
	def createSaFeatureQueryService(node: Node, callbackFunc: Callable[[QueryFeature.Request, QueryFeature.Response], QueryFeature.Response]) -> Service:
		return node.create_service(QueryFeature, SaMsgs.__MAP_SERVICE_NAME, callbackFunc)

	@staticmethod
	def createSaFeatureQueryClient(node: Node) -> Client:
		return node.create_client(QueryFeature, SaMsgs.__MAP_SERVICE_NAME)
