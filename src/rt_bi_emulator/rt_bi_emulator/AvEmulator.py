import json
from math import inf
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sa_msgs.msg import RobotState

import rt_bi_utils.Ros as RosUtils
from rt_bi_emulator.Shared import Target
from rt_bi_interfaces.msg import EstimationMsg, PoseEstimation
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RtBiEmulator import RtBiEmulator
from rt_bi_utils.SaMsgs import SaMsgs


class Av:
	def __init__(self, id: int, cOR: Pose, fov: Geometry.CoordsList) -> None:
		self.id: int = id
		self.centerOfRotation = cOR
		self.interior = fov

	def __repr__(self) -> str:
		name = "#%d" % self.id
		return "AV-%s(COR:%s):%s" % (name, repr(self.centerOfRotation), repr(self.interior))

class AvEmulator(Node):
	""" The Viewer ROS Node """
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__(node_name="rt_bi_emulator_av") # type: ignore - parameter_overrides: List[Parameter] = None
		self.get_logger().debug("%s is initializing." % self.get_fully_qualified_name())
		self.__declareParameters()
		RosUtils.SetLogger(self.get_logger())
		self.__id = -1
		self.__updateInterval = 1
		self.__initTime = float(self.get_clock().now().nanoseconds)
		self.__cutOffTime: float = inf
		self.__centersOfRotation: Geometry.CoordsList = []
		self.__configFilePositions: List[Av] = []
		self.__initFovPoly = Polygon()
		self.__totalTimeNanoSecs = 0.0
		self.__transformationMatrix: List[AffineTransform] = []
		self.__targetIds = set()
		self.__parseConfigFileParameters()
		(self.__fovPublisher, _) = SaMsgs.createRobotStatePublisher(self, self.__publishMyFov, self.__updateInterval)
		(self.__estPublisher, _) = RtBiEmulator.createEstimationPublisher(self)
		RtBiEmulator.subscribeToTargetTopic(self, self.__onTargetUpdate)
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
			self.get_logger().info("parsed %s" % repr(fov))
			parsedFov.append(fov)
			pose = json.loads(saPoses[i])
			timeNanoSecs = int(timePoints[i] * self.NANO_CONVERSION_CONSTANT)
			av = Av(self.__id, Pose(timeNanoSecs, *(pose)), parsedFov[i])
			self.get_logger().info("parsed %s" % repr(av))
			self.__configFilePositions.append(av)
			if i > 0:
				matrix = Geometry.getAffineTransformation(self.__configFilePositions[i - 1].interior, self.__configFilePositions[i].interior)
				self.__transformationMatrix.append(matrix)
		self.__initFovPoly = Polygon(self.__configFilePositions[0].interior)
		self.__totalTimeNanoSecs = timePoints[-1] * self.NANO_CONVERSION_CONSTANT
		return

	def __getFovAtTime(self, timeNanoSecs: int) -> Polygon:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / AvEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			return Geometry.applyMatrixTransformToPolygon(matrix, self.__initFovPoly)
		return Polygon(self.__configFilePositions[-1].interior)

	def __getPoseAtTime(self, timeNanoSecs: int) -> Pose:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / AvEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			return Geometry.applyMatrixTransformToPose(matrix, self.__configFilePositions[0].centerOfRotation)
		return self.__configFilePositions[-1].centerOfRotation

	def __publishMyFov(self) -> None:
		timeOfPublish: int = self.get_clock().now().nanoseconds
		fov = self.__getFovAtTime(timeOfPublish)

		robotStateMsg = RobotState()
		robotStateMsg.robot_id = self.__id

		robotStateMsg.pose = SaMsgs.createSaPoseMsg(self.__configFilePositions[0].centerOfRotation.x, self.__configFilePositions[0].centerOfRotation.y)
		robotStateMsg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))

		self.get_logger().info("Sending updated fov location for AV#%d." % self.__id)
		self.__fovPublisher.publish(robotStateMsg)
		return

	def __onTargetUpdate(self, msg: RobotState) -> None:
		if msg is None:
			self.get_logger().warn("Received empty Target message!")
			return

		coords = SaMsgs.convertSaPoseListToCoordsList(msg.fov.corners)
		timeNanoSecs = self.get_clock().now().nanoseconds
		target = Target(msg.robot_id, Pose(timeNanoSecs, msg.pose.x, msg.pose.y, msg.pose.yaw), coords)
		targetLocation = Polygon(target.spatialRegion)
		fov = self.__getFovAtTime(timeNanoSecs)
		if Geometry.intersects(targetLocation, fov):
			estMsg = EstimationMsg()
			estMsg.detection_time = float(timeNanoSecs)
			robotStateMsg = RobotState()
			robotStateMsg.robot_id = self.__id
			pose = self.__getPoseAtTime(timeNanoSecs)
			robotStateMsg.pose = SaMsgs.createSaPoseMsg(pose.x, pose.y)
			robotStateMsg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
			estMsg.robot_state = robotStateMsg
			poseEstMsg = PoseEstimation(spawned=False, vanished=False)
			if target.id not in self.__targetIds:
				self.get_logger().info("TG#%d spawned." % msg.robot_id)
				poseEstMsg.spawned = True
				self.__targetIds.add(target.id)
			poseEstMsg.trajectory_id = target.id
			poseEstMsg.pose = SaMsgs.createSaPoseMsg(target.location.x, target.location.y, target.location.angleFromX)
			RosUtils.AppendMessage(estMsg.pose_estimations, poseEstMsg)
			self.__estPublisher.publish(estMsg)
			return
		if target.id in self.__targetIds:
			self.get_logger().info("TG#%d vanished." % msg.robot_id)
			estMsg = EstimationMsg()
			estMsg.detection_time = float(timeNanoSecs)
			robotStateMsg = RobotState()
			robotStateMsg.robot_id = self.__id
			pose = self.__getPoseAtTime(timeNanoSecs)
			robotStateMsg.pose = SaMsgs.createSaPoseMsg(pose.x, pose.y)
			robotStateMsg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
			estMsg.robot_state = robotStateMsg
			poseEstMsg = PoseEstimation(spawned=False, vanished=False)
			poseEstMsg.vanished = True
			self.__targetIds.remove(target.id)
			poseEstMsg.trajectory_id = target.id
			poseEstMsg.pose = SaMsgs.createSaPoseMsg(target.location.x, target.location.y, target.location.angleFromX)
			RosUtils.AppendMessage(estMsg.pose_estimations, poseEstMsg)
			self.__estPublisher.publish(estMsg)
			return
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
