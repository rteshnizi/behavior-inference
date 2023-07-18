from typing import Tuple
import rclpy
from rclpy.node import Node

import rt_bi_utils.Ros as RosUtils
from rt_bi_emulator.Case1 import Case1
from rt_bi_utils.Geometry import Geometry, Polygon
from rt_bi_utils.SaMsgs import SaMsgs
from sa_msgs.msg import RobotState


class AvEmulator(Node):
	""" The Viewer ROS Node """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_av_emulator")
		self.get_logger().info("AV Emulator is starting...")
		RosUtils.SetLogger(self.get_logger())
		(self.__fovPublisher, _) = SaMsgs.createSaRobotStatePublisher(self, self.__publishMyFov, 1)
		self.__av1 = Case1.Avs.pop()
		self.__av2 = Case1.Avs.pop()
		self.__initTime = float(self.get_clock().now().nanoseconds)
		self.__totalTime = self.__av1.pose.timeNanoSecs
		self.__initialFovPoly = Polygon(self.__av1.fov)
		self.__centerOfRotation = Case1.CENTER_OF_ROTATION
		self.__transformationMatrix = Geometry.getAffineTransformation(self.__initialFovPoly, Polygon(self.__av2.fov), self.__centerOfRotation)

	def __publishMyFov(self) -> None:
		timeOfPublish = float(self.get_clock().now().nanoseconds)
		param = (timeOfPublish - self.__initTime) / self.__totalTime
		param = 1 if param > 1 else param
		if param < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix, param)
			fov = Geometry.applyMatrixTransformToPolygon(matrix, self.__initialFovPoly, self.__centerOfRotation)
		else:
			fov = Polygon(self.__av2.fov)
		msg = RobotState()
		msg.robot_id = self.__av1.robotId
		msg.pose = SaMsgs.createSaPoseMsg(self.__av1.pose.x, self.__av1.pose.y)
		msg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
		self.get_logger().info("Sending updated fov location for robot %d @ param = %.3f" % (self.__av1.robotId, param))
		self.__fovPublisher.publish(msg)
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	avNode = AvEmulator()
	rclpy.spin(avNode)
	avNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
