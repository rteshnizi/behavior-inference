import json
from math import inf
from typing import List, Set

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sa_msgs.msg import RobotState

import rt_bi_utils.Ros as RosUtils
from rt_bi_emulator.Shared import Target
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RtBiEmulator import RtBiEmulator
from rt_bi_utils.SaMsgs import SaMsgs


class TargetEmulator(Node):
	""" The Viewer ROS Node """
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__(node_name="rt_bi_emulator_target") # type: ignore - parameter_overrides: List[Parameter] = None
		self.get_logger().debug("%s is initializing." % self.get_fully_qualified_name())
		self.__declareParameters()
		RosUtils.SetLogger(self.get_logger())
		self.__id = -1
		self.__updateInterval = 1
		self.__initTime = float(self.get_clock().now().nanoseconds)
		self.__cutOffTime: float = inf
		self.__centersOfRotation: Geometry.CoordsList = []
		self.__targetPositions: List[Target] = []
		self.__initRegionPoly = Polygon()
		self.__totalTimeNanoSecs = 0.0
		self.__transformationMatrix: List[AffineTransform] = []
		self.__parseConfigFileParameters()
		(self.__targetPublisher, _) = RtBiEmulator.createTargetPublisher(self, self.__publishMyRegion, self.__updateInterval)
		# Provides a debugging way to stop the updating the position any further.
		self.__passedCutOffTime: int = -1

	def __declareParameters(self) -> None:
		self.get_logger().debug("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("id", Parameter.Type.INTEGER)
		self.declare_parameter("updateInterval", Parameter.Type.DOUBLE)
		self.declare_parameter("timesSecs", Parameter.Type.DOUBLE_ARRAY)
		self.declare_parameter("cutOffTime", Parameter.Type.DOUBLE)
		self.declare_parameter("saPoses", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("centersOfRotation", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("fovs", Parameter.Type.STRING_ARRAY)
		return

	def __parseConfigFileParameters(self) -> None:
		self.get_logger().debug("%s is parsing parameters." % self.get_fully_qualified_name())
		self.__id = self.get_parameter("id").get_parameter_value().integer_value
		self.__updateInterval = self.get_parameter("updateInterval").get_parameter_value().double_value
		try: self.__cutOffTime = self.get_parameter("cutOffTime").get_parameter_value().double_value
		except: self.__cutOffTime = inf
		timePoints = self.get_parameter("timesSecs").get_parameter_value().double_array_value
		saPoses = list(self.get_parameter("saPoses").get_parameter_value().string_array_value)
		centersOfRotation = list(self.get_parameter("centersOfRotation").get_parameter_value().string_array_value)
		fovs = list(self.get_parameter("fovs").get_parameter_value().string_array_value)
		parsedFov = []
		for i in range(len(timePoints)):
			self.__centersOfRotation.append(json.loads(centersOfRotation[i]))
			fov = [tuple(p) for p in json.loads(fovs[i])]
			self.get_logger().debug("parsed %s" % repr(fov))
			parsedFov.append(fov)
			pose = json.loads(saPoses[i])
			timeNanoSecs = int(timePoints[i] * self.NANO_CONVERSION_CONSTANT)
			av = Target(self.__id, Pose(timeNanoSecs, *(pose)), parsedFov[i])
			self.get_logger().debug("parsed %s" % repr(av))
			self.__targetPositions.append(av)
			if i > 0:
				matrix = Geometry.getAffineTransformation(self.__targetPositions[i - 1].spatialRegion, self.__targetPositions[i].spatialRegion)
				self.__transformationMatrix.append(matrix)
		self.__initRegionPoly = Polygon(self.__targetPositions[0].spatialRegion)
		self.__totalTimeNanoSecs = timePoints[-1] * self.NANO_CONVERSION_CONSTANT
		return

	def __getPoseAtTime(self, timeNanoSecs: int) -> Pose:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / TargetEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			transformed = Geometry.applyMatrixTransformToPose(matrix, self.__targetPositions[0].location)
			transformed.timeNanoSecs = timeNanoSecs
			return transformed
		return Pose(timeNanoSecs, self.__targetPositions[-1].location.x, self.__targetPositions[-1].location.y, self.__targetPositions[-1].location.angleFromX)

	def __publishMyRegion(self) -> None:
		timeOfPublish = self.get_clock().now().nanoseconds
		if self.__passedCutOffTime < 0 and ((timeOfPublish - self.__initTime) / TargetEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeOfPublish
		elif self.__passedCutOffTime > 0:
			timeOfPublish = self.__passedCutOffTime
		elapsedTimeRatio = (timeOfPublish - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			fov = Geometry.applyMatrixTransformToPolygon(matrix, self.__initRegionPoly)
		else:
			fov = Polygon(self.__targetPositions[-1].spatialRegion)
		msg = RobotState()
		msg.robot_id = self.__id
		currentPose = self.__getPoseAtTime(timeOfPublish)
		msg.pose = SaMsgs.createSaPoseMsg(currentPose.x, currentPose.y, currentPose.angleFromX)
		msg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
		self.get_logger().info("Sending location for TG#%d @ %s" % (self.__id, repr(currentPose)))
		self.__targetPublisher.publish(msg)
		return

def main(args=None):
	rclpy.init(args=args)
	targetNode = TargetEmulator()
	rclpy.spin(targetNode)
	targetNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
