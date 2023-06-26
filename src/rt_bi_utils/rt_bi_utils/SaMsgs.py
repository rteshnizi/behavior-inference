# import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry
from sa_msgs.msg import Pose as SaPoseMsg
from sa_msgs.msg import PoseArray as SaPoseArray


class SaMsgs:
	"""
		This class only prepares the messages defined in the TAMU AGC SA project.
	"""

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
