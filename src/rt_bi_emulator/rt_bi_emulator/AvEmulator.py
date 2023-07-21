import json
from typing import Dict, List, Literal, Tuple, Union

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

import rt_bi_utils.Ros as RosUtils
from rt_bi_utils.Geometry import Geometry, Polygon, AffineTransform
from rt_bi_utils.Pose import Pose
from rt_bi_utils.SaMsgs import SaMsgs
from sa_msgs.msg import RobotState


class Av:
	def __init__(self, id: int, p: Pose, fov: Geometry.CoordsList) -> None:
		self.robotId: int = id
		self.pose = p
		self.fov = fov

class AvEmulator(Node):
	""" The Viewer ROS Node """
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_av_emulator")
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		self.__declareParameters()
		RosUtils.SetLogger(self.get_logger())
		self.__centerOfRotation = (0.0, 0.0)
		self.__robotId = -1
		self.__initTime = float(self.get_clock().now().nanoseconds)
		self.__avPositions: List[Av] = []
		self.__initFovPoly = Polygon()
		self.__totalTimeNanoSecs = 0.0
		self.__transformationMatrix = AffineTransform()
		self.__angularVelocity = 1.0
		self.__parseConfigFileParameters()
		(self.__fovPublisher, _) = SaMsgs.createSaRobotStatePublisher(self, self.__publishMyFov, (1 / self.__angularVelocity))

	def __declareParameters(self) -> None:
		self.get_logger().info("%s is setting node parameters..." % self.get_fully_qualified_name())
		self.declare_parameter("robotId", Parameter.Type.INTEGER)
		self.declare_parameter("angularVelocity", Parameter.Type.DOUBLE)
		self.declare_parameter("timeSecs", Parameter.Type.DOUBLE_ARRAY),
		self.declare_parameter("saPose", Parameter.Type.STRING_ARRAY),
		self.declare_parameter("fov", Parameter.Type.STRING_ARRAY),
		return

	def __parseConfigFileParameters(self) -> None:
		self.get_logger().info("%s is parsing parameters..." % self.get_fully_qualified_name())
		self.__robotId = self.get_parameter("robotId").get_parameter_value().integer_value
		self.__angularVelocity = self.get_parameter_or("angularVelocity").get_parameter_value().double_value
		timePoints = self.get_parameter("timeSecs").get_parameter_value().double_array_value
		saPoses = self.get_parameter("saPose").get_parameter_value().string_array_value
		saPoses = [json.loads(pose) for pose in saPoses]
		saPoses = tuple(saPoses)
		self.__centerOfRotation = (saPoses[0][0], saPoses[0][1])
		fov = self.get_parameter("fov").get_parameter_value().string_array_value
		parsedFov = []
		for f in fov:
			f = json.loads(f)
			parsedFov.append([tuple(p) for p in f])
		for i in range(len(timePoints)):
			self.__avPositions.append(Av(self.__robotId, Pose(timePoints[i], *(saPoses[0])), parsedFov[i]))
		self.__initFovPoly = Polygon(self.__avPositions[0].fov)
		polygon1 = Polygon(self.__avPositions[-1].fov)
		self.__transformationMatrix = Geometry.getAffineTransformation(self.__initFovPoly, polygon1, self.__centerOfRotation)
		self.__totalTimeNanoSecs = timePoints[-1] * AvEmulator.NANO_CONVERSION_CONSTANT
		return

	def __publishMyFov(self) -> None:
		timeOfPublish = float(self.get_clock().now().nanoseconds)
		param = (timeOfPublish - self.__initTime) / self.__totalTimeNanoSecs
		param = 1 if param > 1 else param
		if param < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix, param)
			fov = Geometry.applyMatrixTransformToPolygon(matrix, self.__initFovPoly, self.__centerOfRotation)
		else:
			fov = Polygon(self.__avPositions[-1].fov)
		msg = RobotState()
		msg.robot_id = self.__robotId
		msg.pose = SaMsgs.createSaPoseMsg(self.__avPositions[0].pose.x, self.__avPositions[0].pose.y)
		msg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
		self.get_logger().info("Sending updated fov location for robot %d @ param = %.3f" % (self.__robotId, param))
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
