import json
from abc import ABC
from math import inf

from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Pose import Pose
from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_commons.Utils.Msgs import Msgs


class Body:
	def __init__(self, id: str, location: Pose, centerOfRotation: GeometryLib.Coords, spatialRegion: GeometryLib.CoordsList) -> None:
		self.id: str = id
		self.location = location
		self.spatialRegion = spatialRegion
		self.centerOfRotation = centerOfRotation

	def __repr__(self) -> str:
		return f"#{self.id}"

class AffineRegionEmulator(RtBiNode, ABC):
	"""
	This class provides a ROS node which emulates a moving polygonal region.
	It provides the base functionality to read config files and
	an API to prepare updates about the new state of the region.

	**NOTICE**, this class does not contain any publishers. The relevant publishers has to be made in ``__init__`` of the subclasses.
	"""
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self, **kwArgs):
		newKw = { "node_name": "emulator_region_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.id = ""
		self.updateInterval = 1
		self.__initTime = float(self.get_clock().now().nanoseconds)
		self.__cutOffTime: float = inf
		self.__centersOfRotation: GeometryLib.CoordsList = []
		self.regionPositions: list[Body] = []
		self.__initialRegionPoly = Shapely.Polygon()
		self.__totalTimeNanoSecs = 0.0
		self.__transformationMatrix: list[AffineTransform] = []
		self.parseParameters()
		# Provides a debugging way to stop the updating the position any further.
		self.__passedCutOffTime: int = -1

	def __getCenterOfRotationAtTime(self, timeNanoSecs: int) -> Pose:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / AffineRegionEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			return Pose(timeNanoSecs, self.regionPositions[0].location.x, self.regionPositions[0].location.y, self.regionPositions[0].location.angleFromX)
		return Pose(timeNanoSecs, self.regionPositions[-1].location.x, self.regionPositions[-1].location.y, self.regionPositions[-1].location.angleFromX)

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("id", Parameter.Type.STRING)
		self.declare_parameter("updateInterval", Parameter.Type.DOUBLE)
		self.declare_parameter("timesSecs", Parameter.Type.DOUBLE_ARRAY)
		self.declare_parameter("cutOffTime", Parameter.Type.DOUBLE)
		self.declare_parameter("saPoses", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("centersOfRotation", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("fovs", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		self.id = str(self.get_parameter("id").get_parameter_value().string_value)
		self.updateInterval = self.get_parameter("updateInterval").get_parameter_value().double_value
		try: self.__cutOffTime = self.get_parameter("cutOffTime").get_parameter_value().double_value
		except: self.__cutOffTime = inf
		timePoints = self.get_parameter("timesSecs").get_parameter_value().double_array_value
		saPoses = list(self.get_parameter("saPoses").get_parameter_value().string_array_value)
		centersOfRotation = list(self.get_parameter("centersOfRotation").get_parameter_value().string_array_value)
		fovs = list(self.get_parameter("fovs").get_parameter_value().string_array_value)
		for i in range(len(timePoints)):
			self.__centersOfRotation.append(json.loads(centersOfRotation[i]))
			fov = [tuple(p) for p in json.loads(fovs[i])]
			pose = json.loads(saPoses[i])
			timeNanoSecs = int(timePoints[i] * self.NANO_CONVERSION_CONSTANT)
			body = Body(self.id, Pose(timeNanoSecs, *(pose)), self.__centersOfRotation[i], fov)
			self.log(f"Parsed {repr(body)} config @ {timeNanoSecs}")
			self.regionPositions.append(body)
			if i > 0:
				matrix = GeometryLib.getAffineTransformation(self.regionPositions[i - 1].spatialRegion, self.regionPositions[i].spatialRegion)
				self.__transformationMatrix.append(matrix)
		self.__initialRegionPoly = Shapely.Polygon(self.regionPositions[0].spatialRegion)
		self.__totalTimeNanoSecs = timePoints[-1] * self.NANO_CONVERSION_CONSTANT
		return

	def getRegionAtTime(self, timeNanoSecs: int) -> Shapely.Polygon:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / AffineRegionEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = GeometryLib.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			return GeometryLib.applyMatrixTransformToPolygon(matrix, self.__initialRegionPoly)
		return Shapely.Polygon(self.regionPositions[-1].spatialRegion)

	def getPoseAtTime(self, timeNanoSecs: int) -> Pose:
		if self.__passedCutOffTime < 0 and ((timeNanoSecs - self.__initTime) / AffineRegionEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTime:
			self.__passedCutOffTime = timeNanoSecs
		elif self.__passedCutOffTime > 0:
			timeNanoSecs = self.__passedCutOffTime
		elapsedTimeRatio = (timeNanoSecs - self.__initTime) / self.__totalTimeNanoSecs
		elapsedTimeRatio = 1 if elapsedTimeRatio > 1 else elapsedTimeRatio
		if elapsedTimeRatio < 1:
			matrix = GeometryLib.getParameterizedAffineTransformation(self.__transformationMatrix[0], elapsedTimeRatio)
			transformed = GeometryLib.applyMatrixTransformToPose(matrix, self.regionPositions[0].location)
			transformed.timeNanoSecs = timeNanoSecs
			return transformed
		return Pose(timeNanoSecs, self.regionPositions[-1].location.x, self.regionPositions[-1].location.y, self.regionPositions[-1].location.angleFromX)

	def asRegularSpaceMsg(self) -> Msgs.RtBi.RegularSpace:
		timeOfPublish = self.get_clock().now()
		poly = self.getRegionAtTime(timeOfPublish.nanoseconds)
		currentCor = self.__getCenterOfRotationAtTime(timeOfPublish.nanoseconds)
		polyMsg = Msgs.RtBi.Polygon()
		polyMsg.region = Msgs.toStdPolygon(poly)
		polyMsg.id = "0" # All emulated dynamic regions currently only hold a single polygon.
		polyMsg.center_of_rotation = Msgs.toStdPoint(currentCor)
		msg = Msgs.RtBi.RegularSpace()
		msg.id = self.id
		msg.stamp = timeOfPublish.to_msg()
		msg.polygons = [polyMsg]
		msg.ros_node = self.get_fully_qualified_name()
		msg.predicates = [] # Inherit all other predicates
		return msg

	def asRegularSpaceArrayMsg(self) -> Msgs.RtBi.RegularSpaceArray:
		msg = self.asRegularSpaceMsg()
		arr = Msgs.RtBi.RegularSpaceArray()
		arr.spaces = [msg]
		return arr

	def render(self) -> None:
		self.log(f"No render for {self.get_fully_qualified_name()}.")
		return
