import json
from abc import ABC
from math import inf
from typing import Any, Generic, TypeVar

from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter

from rt_bi_commons.Base.RtBiNode import RtBiNode
from rt_bi_commons.Shared.Pose import Coords
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_core.Spatial import PolygonFactory, PolygonFactoryKeys
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.ContinuousTimePolygon import ContinuousTimePolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.TargetPolygon import TargetPolygon

_T_Poly = TypeVar("_T_Poly", AffinePolygon, SensingPolygon, TargetPolygon )
class AffineRegionEmulator(Generic[_T_Poly], RtBiNode, ABC):
	"""
	This class provides a ROS node which emulates a moving polygonal region.
	It provides the base functionality to read config files and
	an API to prepare updates about the new state of the region.

	**NOTICE**, this class does not contain any publishers. The relevant publishers has to be made in ``__init__`` of the subclasses.
	"""
	NANO_CONVERSION_CONSTANT = 10 ** 9
	def __init__(self, PolyCls: type[_T_Poly], **kwArgs):
		newKw = { "node_name": "emulator_region_base", "loggingSeverity": LoggingSeverity.INFO, **kwArgs}
		super().__init__(**newKw)
		self.declareParameters()
		self.id = ""
		self.updateInterval = 1
		self.__polyClass = PolyCls
		self.__initTimeNs: int = self.get_clock().now().nanoseconds
		self.__cutOffTimeSecs: float = inf
		self.__ctPoly: ContinuousTimePolygon[PolyCls] = ContinuousTimePolygon([])
		self.__predicates = Predicates([])
		self.parseParameters()

	def declareParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is setting node parameters.")
		self.declare_parameter("id", Parameter.Type.STRING)
		self.declare_parameter("updateInterval", Parameter.Type.DOUBLE)
		self.declare_parameter("timesSecs", Parameter.Type.DOUBLE_ARRAY)
		self.declare_parameter("cutOffTime", Parameter.Type.DOUBLE)
		self.declare_parameter("centersOfRotation", Parameter.Type.STRING_ARRAY)
		self.declare_parameter("polygons", Parameter.Type.STRING_ARRAY)
		return

	def parseParameters(self) -> None:
		self.log(f"{self.get_fully_qualified_name()} is parsing parameters.")
		self.id = str(self.get_parameter("id").get_parameter_value().string_value)
		self.updateInterval = self.get_parameter("updateInterval").get_parameter_value().double_value
		try: self.__cutOffTimeSecs = self.get_parameter("cutOffTime").get_parameter_value().double_value
		except: self.__cutOffTimeSecs = inf
		timePoints = self.get_parameter("timesSecs").get_parameter_value().double_array_value
		centersOfRotation = list(self.get_parameter("centersOfRotation").get_parameter_value().string_array_value)
		polygons = list(self.get_parameter("polygons").get_parameter_value().string_array_value)
		self.__ctPoly = ContinuousTimePolygon([])
		for i in range(len(timePoints)):
			cor = json.loads(centersOfRotation[i])
			poly = [tuple(p) for p in json.loads(polygons[i])]
			timeNanoSecs = int(timePoints[i] * self.NANO_CONVERSION_CONSTANT) + self.__initTimeNs

			kwArgs: dict[PolygonFactoryKeys, Any] = {
				"polygonId": "0",
				"regionId": self.id,
				"subPartId": "",
				"envelope": poly,
				"predicates": self.__predicates,
				"centerOfRotation": cor,
				"timeNanoSecs": timeNanoSecs,
				"hIndex": -1,
			}
			poly = PolygonFactory(self.__polyClass, kwArgs)
			self.log(f"Parsed {poly} config @ {timeNanoSecs}")
			self.__ctPoly.addPolygon(poly)
		return

	def getRegionAtTime(self, timeNanoSecs: int) -> _T_Poly:
		if ((timeNanoSecs - self.__initTimeNs) / AffineRegionEmulator.NANO_CONVERSION_CONSTANT) > self.__cutOffTimeSecs:
			timeNanoSecs = int(self.__cutOffTimeSecs * AffineRegionEmulator.NANO_CONVERSION_CONSTANT)
		if timeNanoSecs in self.__ctPoly: return self.__ctPoly[timeNanoSecs]
		Ros.Logger().warn(f"{self.get_fully_qualified_name()} stream ended.")
		return self.__ctPoly.configs[-1]

	def asRegularSpaceMsg(self, timeNanoSecs: int) -> Msgs.RtBi.RegularSet:
		self.log(f"{self.id} is publishing @ {timeNanoSecs}")
		poly = self.getRegionAtTime(timeNanoSecs)
		polyMsg = Msgs.RtBi.Polygon()
		polyMsg.region = Msgs.toPolygonMsg(poly.interior)
		polyMsg.id = "0" # All emulated dynamic regions currently only hold a single polygon.
		polyMsg.center_of_rotation = Msgs.toPointMsg(poly.centerOfRotation)
		msg = Msgs.RtBi.RegularSet()
		msg.id = self.id
		msg.stamp = Msgs.toTimeMsg(timeNanoSecs)
		msg.polygons = [polyMsg]
		msg.channel = self.get_fully_qualified_name()
		msg.predicates = [] # Inherit all other predicates
		msg.set_type= self.__ctPoly.type.value
		return msg

	def render(self) -> None:
		self.log(f"No render for {self.get_fully_qualified_name()}.")
		return
