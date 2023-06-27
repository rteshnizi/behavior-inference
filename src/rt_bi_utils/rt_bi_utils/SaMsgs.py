from typing import Callable, Union
from rclpy.node import Client, Node, Service

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry
from sa_msgs.msg import Pose as SaPoseMsg
from sa_msgs.msg import PoseArray as SaPoseArray
from sa_msgs.msg import FeatureInfo
from sa_msgs.srv import QueryFeature


class SaMsgs:
	"""
		This class only prepares the messages defined in the TAMU AGC SA project.
	"""
	__MAP_UPDATE_TOPIC = "/sa_map/FeatureMap_BIL"
	__MAP_SERVICE_NAME = "/sa_map/feature_query"

	@staticmethod
	def createSaPoseMessage(x: float, y: float, yaw = 0.0) -> SaPoseMsg:
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
	def createSaPoseArrayMessage(coords: Geometry.CoordsList) -> SaPoseArray:
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
			message.traj.append(SaMsgs.createSaPoseMessage(*coord))
		return message

	@staticmethod
	def subscribeToMapUpdateTopic(node: Node, callbackFunc: Callable[[FeatureInfo], None]) -> None:
		RosUtils.CreateSubscriber(node, FeatureInfo, SaMsgs.__MAP_UPDATE_TOPIC, callbackFunc)

	@staticmethod
	def createFeatureQueryService(node: Node, callbackFunc: Callable[[QueryFeature.Request, QueryFeature.Response], QueryFeature.Response]) -> Service:
		return node.create_service(QueryFeature, SaMsgs.__MAP_SERVICE_NAME, callbackFunc)

	@staticmethod
	def createFeatureQueryClient(node: Node) -> Client:
		return node.create_client(QueryFeature, SaMsgs.__MAP_SERVICE_NAME)

	@staticmethod
	def createFeatureQueryClient(node: Node) -> Client:
		return node.create_client(QueryFeature, SaMsgs.__MAP_SERVICE_NAME)
