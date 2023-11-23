from math import isnan, nan
from typing import Generic, List, Sequence, Tuple, TypeVar

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import AffineTransform, Geometry
from rt_bi_utils.Ros import Logger

RegionType = TypeVar("RegionType", bound=AffineRegion)

class ContinuousTimeRegion(Generic[RegionType]):
	def __init__(self, *regionConfigs: RegionType) -> None:
		self.__configs: Tuple[RegionType, ...] = ()
		self.__configs = tuple(sorted(regionConfigs, key=lambda r: r.timeNanoSecs))
		self.__transforms: Sequence[AffineTransform] = self.__obtainTransformationMatrix()
		Logger().debug("Created %s" % repr(self))
		return

	def __repr__(self) -> str:
		return "CTR-%s[%d, %d]" % (self.name, self.earliestNanoSecs, self.latestNanoSecs)

	def __getitem__(self, timeNanoSecs: int) -> RegionType:
		"""Get polygonal shape at time.

		Parameters
		----------
		timeNanoSecs : int
			Time in NanoSeconds.

		Returns
		-------
		Polygon
			The shape.
		"""
		if self.length == 0: raise ValueError("Empty CTR.")
		if self.earliestNanoSecs != -1 and timeNanoSecs < self.earliestNanoSecs:
			return self.configs[0]
		if self.earliestNanoSecs != -1 and timeNanoSecs > self.latestNanoSecs:
			return self.configs[-1]
		i = 0
		for i in range(self.length):
			if timeNanoSecs <= self.configs[i].timeNanoSecs: break
		param = self.__getParameterizedTime(i - 1, timeNanoSecs)
		if isnan(param):
			return self.configs[i - 1]
		transform = Geometry.getParameterizedAffineTransformation(self.__transforms[i - 1], param)
		pose = Geometry.applyMatrixTransformToPose(transform, self.configs[i - 1].centerOfRotation)
		regionPoly = Geometry.applyMatrixTransformToPolygon(transform, self.configs[i - 1].interior)
		if self.regionType == AffineRegion.RegionType.SENSING:
			tracklets: Tracklets = self.configs[i - 1].tracks # type: ignore -- We know it's a sensor
			region = SensorRegion(pose, self.idNum, [], timeNanoSecs, interior=regionPoly, tracks=tracklets)
		elif self.configs[i - 1].regionType == AffineRegion.RegionType.MAP:
			region = MapRegion(0, [], timeNanoSecs, interior=regionPoly)
		elif self.configs[i - 1].regionType == AffineRegion.RegionType.SYMBOL:
			region = SymbolRegion(self.configs[i - 1].centerOfRotation, self.idNum, [], timeNanoSecs, interior=regionPoly)
		else:
			raise TypeError("Unexpected region type: %s" % repr(self.regionType))
		return region # type: ignore -- Type checker doesn't understand I have type-checked here via regionType.

	@property
	def name(self) -> str:
		"""Region name."""
		return "∅" if self.length < 1 else self.configs[0].name

	@property
	def idNum(self) -> int:
		if self.length == 0:
			return -1
		return self.configs[0].idNum

	@property
	def length(self) -> int:
		return len(self.__configs)

	@property
	def configs(self) -> Tuple[RegionType, ...]:
		return self.__configs

	@property
	def regionType(self) -> AffineRegion.RegionType:
		if self.length == 0:
			return AffineRegion.RegionType.BASE
		return self.configs[0].regionType

	@property
	def transformations(self) -> Sequence[AffineTransform]:
		return self.__transforms

	@property
	def latestNanoSecs(self) -> int:
		if self.length == 0:
			return -1
		return self.__configs[-1].timeNanoSecs

	@property
	def earliestNanoSecs(self) -> int:
		if self.length == 0:
			return -1
		return self.__configs[0].timeNanoSecs

	def __getParameterizedTime(self, i: int, timeNanoSecs: int) -> float:
		"""### Get Parameterized Time
		Given a time, returns it as a fraction of the interval between `self.__configs[i]` and `self.__configs[i + 1]`.

		Parameters
		----------
		i : int
			The index of the lower bound of time in `self.__configs`
		timeNanoSecs : int
			ROS time in Nanoseconds

		Returns
		-------
		float
			A number in the range `[0, 1]`, or `nan` if the two ends of the interval are the same.
		"""
		if self.length < 2: return nan
		frac = timeNanoSecs - self.configs[i].timeNanoSecs
		total = self.configs[i + 1].timeNanoSecs - self.configs[i].timeNanoSecs
		if total == 0: return nan
		return (float(frac) / float(total))

	def __obtainTransformationMatrix(self) -> Sequence[AffineTransform]:
		transforms: List[AffineTransform] = []
		if self.length < 2: return transforms
		for i in range(self.length - 1):
			transform = Geometry.getAffineTransformation(self.configs[i].envelope, self.configs[i + 1].envelope)
			transforms.append(transform)
		return transforms
