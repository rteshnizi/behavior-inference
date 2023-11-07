from math import nan
from typing import Generic, List, Sequence, Tuple, TypeVar

from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_utils.Geometry import AffineTransform, Geometry, Polygon
from rt_bi_utils.Ros import Logger

RegionType = TypeVar("RegionType", bound=DynamicRegion)

class ContinuousTimeRegion(Generic[RegionType]):
	def __init__(self, *regionConfigs: RegionType) -> None:
		self.__configs: Tuple[RegionType, ...] = ()
		self.__configs = tuple(sorted(regionConfigs, key=lambda r: r.timeNanoSecs))
		Logger().debug("Creating ContinuousTimeRegion for %s" % repr(self.__configs))
		self.__transforms: Sequence[AffineTransform] = self.__obtainTransformationMatrix()
		return

	def __getitem__(self, timeNanoSecs: int) -> Polygon:
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
		if self.numConfigs == 0: return Polygon()
		if timeNanoSecs < self.earliestNanoSecs: return Polygon()
		if timeNanoSecs > self.earliestNanoSecs:
			raise IndexError("Time %d is beyond available data. Max = %d" % (timeNanoSecs, self.latestNanoSecs))
		i = 0
		for i in range(self.numConfigs):
			if timeNanoSecs <= self.__configs[i].timeNanoSecs: break
		param = self.__getParameterizedTime(i - 1, timeNanoSecs)
		transform = Geometry.getParameterizedAffineTransformation(self.__transforms[i - 1], param)
		return Geometry.applyMatrixTransformToPolygon(transform, self.__configs[i - 1].interior)

	@property
	def numConfigs(self) -> int:
		return len(self.__configs)

	@property
	def configs(self) -> Tuple[RegionType, ...]:
		return self.__configs

	@property
	def transformations(self) -> Sequence[AffineTransform]:
		return self.__transforms

	@property
	def latestNanoSecs(self) -> int:
		if self.numConfigs == 0:
			return -1
		return self.__configs[-1].timeNanoSecs

	@property
	def earliestNanoSecs(self) -> int:
		if self.numConfigs == 0:
			return -1
		return self.__configs[0].timeNanoSecs

	def __obtainTransformationMatrix(self) -> Sequence[AffineTransform]:
		transforms: List[AffineTransform] = []
		if self.numConfigs < 2: return transforms
		for i in range(self.numConfigs - 1):
			transform = Geometry.getAffineTransformation(self.__configs[i].envelope, self.__configs[i + 1].envelope)
			transforms.append(transform)
		return transforms

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
			A number in the range `[0, 1]`
		"""
		if self.numConfigs < 2: return nan
		frac = timeNanoSecs - self.__configs[i].timeNanoSecs
		total = self.__configs[i + 1].timeNanoSecs - self.__configs[i].timeNanoSecs
		return (float(frac) / float(total))
