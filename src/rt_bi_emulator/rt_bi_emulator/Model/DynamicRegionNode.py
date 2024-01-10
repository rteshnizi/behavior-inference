import json
from math import inf
from typing import List

from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter
from sa_msgs.msg import RobotState

from rt_bi_emulator.Model.Shared import Body
from rt_bi_interfaces.msg import DynamicRegion as DynamicRegionMsg
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RtBiInterfaces import RtBiInterfaces
from rt_bi_utils.RtBiNode import RtBiNode
from rt_bi_utils.SaMsgs import SaMsgs


class DynamicRegionNode(RtBiNode):
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self, loggingSeverity: LoggingSeverity, **kwArgs):
		newKw = { "node_name": "rt_bi_emulator_dyn_region_base", "loggingSeverity": loggingSeverity, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.id = -1
		self.updateInterval = 1
		self.__initTime = float(self.get_clock().now().nanoseconds)
		self.__cutOffTime: float = inf
		self.__centersOfRotation: Geometry.CoordsList = []
		self.regionPositions: List[Body] = []
		self.__initialRegionPoly = Polygon()
		self.__totalTimeNanoSecs = 0.0
		self.__transformationMatrix: List[AffineTransform] = []
		self.parseConfigFileParameters()
		# Provides a debugging way to stop the updating the position any further.
		self.__passedCutOffTime: int = -1

	def declareParameters(self) -> None:
		self.log("%s is setting node parameters." % self.get_fully_qualified_name())
		self.declare_parameter("id", Parameter.Type.INTEGER)
		self.declare_parameter("updateInterval", Parameter.Type.DOUBLE)
		self.declare_parameter("timesSecs", Parameter.Type.DOUBLE_ARRAY)
		self.declare_parameter("cutOffTime", Parameter.Type.DOUBLE)
		self.declare_parameter("saPoses", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("centersOfRotation", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("fovs", Parameter.Type.STRING_ARRAY)
		return

	def parseConfigFileParameters(self) -> None:
		self.log("%s is parsing parameters." % self.get_fully_qualified_name())
		self.id = self.get_parameter("id").get_parameter_value().integer_value
		self.updateInterval = self.get_parameter("updateInterval").get_parameter_value().double_value
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
			self.log("parsed %s" % repr(fov))
			parsedFov.append(fov)
			pose = json.loads(saPoses[i])
			timeNanoSecs = int(timePoints[i] * self.NANO_CONVERSION_CONSTANT)
			body = Body(self.id, Pose(timeNanoSecs, *(pose)), self.__centersOfRotation[i], parsedFov[i])
			self.log("parsed %s" % repr(body))
			self.regionPositions.append(body)
			if i > 0:
				matrix = Geometry.getAffineTransformation(self.regionPositions[i - 1].spatialRegion, self.regionPositions[i].spatialRegion)
				self.__transformationMatrix.append(matrix)
		self.__initialRegionPoly = Polygon(self.regionPositions[0].spatialRegion)
		self.__totalTimeNanoSecs = timePoints[-1] * self.NANO_CONVERSION_CONSTANT
		return

	def getRegionAtTime(self, timeNanoSecs: int) -> Polygon:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / DynamicRegionNode.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			return Geometry.applyMatrixTransformToPolygon(matrix, self.__initialRegionPoly)
		return Polygon(self.regionPositions[-1].spatialRegion)

	def getPoseAtTime(self, timeNanoSecs: int) -> Pose:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / DynamicRegionNode.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			transformed = Geometry.applyMatrixTransformToPose(matrix, self.regionPositions[0].location)
			transformed.timeNanoSecs = timeNanoSecs
			return transformed
		return Pose(timeNanoSecs, self.regionPositions[-1].location.x, self.regionPositions[-1].location.y, self.regionPositions[-1].location.angleFromX)

	def __getCenterOfRotationAtTime(self, timeNanoSecs: int) -> Pose:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / DynamicRegionNode.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			return Pose(timeNanoSecs, self.regionPositions[0].location.x, self.regionPositions[0].location.y, self.regionPositions[0].location.angleFromX)
		return Pose(timeNanoSecs, self.regionPositions[-1].location.x, self.regionPositions[-1].location.y, self.regionPositions[-1].location.angleFromX)

	def createDynamicRegionMsg(self) -> DynamicRegionMsg:
		timeOfPublish = self.get_clock().now()
		region = self.getRegionAtTime(timeOfPublish.nanoseconds)
		currentPose = self.getPoseAtTime(timeOfPublish.nanoseconds)
		currentCor = self.__getCenterOfRotationAtTime(timeOfPublish.nanoseconds)
		msg = DynamicRegionMsg(
			id=self.id,
			stamp=timeOfPublish.to_msg(),
			pose=RtBiInterfaces.toStdPose(currentPose),
			center_of_rotation=RtBiInterfaces.toStdPoint(currentCor),
			region=RtBiInterfaces.toPolygonMsg(region),
		)
		return msg

	def createRobotStateMsg(self) -> RobotState:
		timeOfPublish = self.get_clock().now().nanoseconds
		if self.__passedCutOffTime < 0 and ((timeOfPublish - self.__initTime) / DynamicRegionNode.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeOfPublish
		elif self.__passedCutOffTime > 0:
			timeOfPublish = self.__passedCutOffTime
		elapsedTimeRatio = (timeOfPublish - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = Geometry.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			fov = Geometry.applyMatrixTransformToPolygon(matrix, self.__initialRegionPoly)
		else:
			fov = Polygon(self.regionPositions[-1].spatialRegion)
		msg = RobotState()
		msg.robot_id = self.id
		currentPose = self.getPoseAtTime(timeOfPublish)
		msg.pose = SaMsgs.createSaPoseMsg(currentPose.x, currentPose.y, currentPose.angleFromX)
		msg.fov = SaMsgs.createSaFovMsg(list(fov.exterior.coords))
		return msg

	def render(self) -> None:
		self.log(f"No render for {self.get_fully_qualified_name()}.")
		return
