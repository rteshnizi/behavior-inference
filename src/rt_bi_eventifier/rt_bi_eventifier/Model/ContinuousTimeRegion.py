from math import isnan, nan
from typing import Dict, Generic, List, Sequence, Set, TypeVar, Union

from rt_bi_core.MapRegion import MapRegion
from rt_bi_core.SensorRegion import SensorRegion
from rt_bi_core.SymbolRegion import SymbolRegion
from rt_bi_core.Tracklet import Tracklet
from rt_bi_utils.Geometry import AffineTransform, Geometry, LineString, Polygon
from rt_bi_utils.Ros import Logger

RegionType = TypeVar("RegionType", SensorRegion, SymbolRegion, MapRegion)

class ContinuousTimeRegion(Generic[RegionType]):
	def __init__(self, regionConfigs: Sequence[RegionType], regionType: SensorRegion.RegionType) -> None:
		self.__regionType: SensorRegion.RegionType = regionType
		self.__sortedConfigs: List[RegionType] = list(sorted(regionConfigs, key=lambda r: r.timeNanoSecs))
		self.__transforms: Sequence[AffineTransform] = self.__initTransformationMatrices()
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
		if self.earliestNanoSecs == -1: raise ValueError("earliestNanoSecs is not set.")
		if self.latestNanoSecs == -1: raise ValueError("latestNanoSecs is not set.")
		if timeNanoSecs < self.earliestNanoSecs: raise IndexError(f"{timeNanoSecs} is less than earliestNanoSecs={self.earliestNanoSecs}.")
		if timeNanoSecs > self.latestNanoSecs: raise IndexError(f"{timeNanoSecs} is greater than latestNanoSecs={self.latestNanoSecs}.")
		if timeNanoSecs == self.earliestNanoSecs:
			return self.configs[0]
		if timeNanoSecs == self.latestNanoSecs:
			return self.configs[-1]
		i = 0
		for i in range(self.length):
			if timeNanoSecs <= self.configs[i].timeNanoSecs: break
		param = self.__getParameterizedTime(i - 1, timeNanoSecs)
		if isnan(param):
			return self.configs[i - 1]
		transform = Geometry.getParameterizedAffineTransformation(self.__transforms[i - 1], param)
		cor = Geometry.applyMatrixTransformToCoords(transform, self.configs[i - 1].centerOfRotation)
		regionPoly = Geometry.applyMatrixTransformToPolygon(transform, self.configs[i - 1].interior)
		if self.regionType == SensorRegion.RegionType.SENSING:
			tracklets: Dict[int, Tracklet] = self.configs[i - 1].tracklets # type: ignore
			region = SensorRegion(centerOfRotation=cor, idNum=self.idNum, envelope=[], timeNanoSecs=timeNanoSecs, interior=regionPoly, tracklets=tracklets)
		elif self.configs[i - 1].regionType == MapRegion.RegionType.MAP:
			region = MapRegion(0, [], timeNanoSecs, interior=regionPoly)
		elif self.configs[i - 1].regionType == SymbolRegion.RegionType.SYMBOL:
			region = SymbolRegion(self.configs[i - 1].centerOfRotation, self.idNum, [], timeNanoSecs, overlappingRegionId=self.configs[i - 1].overlappingRegionId, overlappingRegionType=self.configs[i - 1].inFov, interior=regionPoly) # type: ignore
		else:
			raise TypeError("Unexpected region type: %s" % repr(self.regionType))
		return region # type: ignore -- Type checker doesn't understand I have type-checked here via regionType.

	def __contains__(self, timeNanoSecs: int) -> bool:
		if self.earliestNanoSecs == -1: return False
		if self.latestNanoSecs == -1: return False
		if timeNanoSecs < self.earliestNanoSecs: return False
		if timeNanoSecs > self.latestNanoSecs: return False
		return True

	@property
	def name(self) -> str:
		"""Region name."""
		return f"{self.regionType}" if self.length < 1 else self.configs[0].shortName

	@property
	def idNum(self) -> int:
		if self.length == 0:
			return -1
		return self.configs[0].idNum

	@property
	def length(self) -> int:
		return len(self.__sortedConfigs)

	@property
	def configs(self) -> List[RegionType]:
		return self.__sortedConfigs

	@property
	def regionType(self) -> SensorRegion.RegionType:
		return self.__regionType

	@property
	def transformations(self) -> Sequence[AffineTransform]:
		return self.__transforms

	@property
	def latestNanoSecs(self) -> int:
		if self.length == 0:
			return -1
		return self.__sortedConfigs[-1].timeNanoSecs

	@property
	def earliestNanoSecs(self) -> int:
		if self.length == 0:
			return -1
		return self.__sortedConfigs[0].timeNanoSecs

	@property
	def isSlice(self) -> bool:
		return self.earliestNanoSecs == self.latestNanoSecs and self.earliestNanoSecs != -1

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

	def __initTransformationMatrices(self) -> Sequence[AffineTransform]:
		transforms: List[AffineTransform] = []
		if self.length < 2: return transforms
		for i in range(self.length - 1):
			transform = Geometry.getAffineTransformation(self.configs[i].envelope, self.configs[i + 1].envelope)
			transforms.append(transform)
		return transforms

	def __getOverallTransformationMatrix(self) -> AffineTransform:
		if self.length < 2: return AffineTransform()
		transform = Geometry.getAffineTransformation(self.configs[0].envelope, self.configs[-1].envelope)
		return transform

	def __timeNanoSecsToIndex(self, timeNanoSecs: int) -> int:
		"""### Time ns to Index

		Parameters
		----------
		timeNanoSecs : int

		Returns
		-------
		int
			The index of the config this time belongs to.

		Raises
		------
		IndexError
			timeNanoSecs is not in range. Test with `timeNanoSecs in continuousTimeRegion`
		"""
		if timeNanoSecs not in self: raise IndexError(f"{timeNanoSecs} is not in range")

		for i in range(self.length):
			if timeNanoSecs > self.configs[i].timeNanoSecs: continue
			return i - 1
		raise IndexError(f"{timeNanoSecs} is both inside the interval [{self.earliestNanoSecs} {self.latestNanoSecs}) and not?")

	def getEdgeBb(self, eName: str) -> Union[Polygon, LineString]:
		"""### Get Edge Bounding Box

		Parameters
		----------
		eName : str
			Name of the edge.
		timeNanoSecs : int

		Returns
		-------
		Union[Polygon, LineString]
		"""
		edge = self.configs[0].edges[eName]
		transform = self.__getOverallTransformationMatrix()
		return Geometry.getLineSegmentExpandedBb(transform, edge, self.configs[0].centerOfRotation)

	def getEdgeAt(self, eName: str, timeNanoSecs: int) -> LineString:
		e = self.configs[0].edges[eName]
		i = self.__timeNanoSecsToIndex(timeNanoSecs)
		frac = self.__getParameterizedTime(i, timeNanoSecs)
		transformationAtT = Geometry.getParameterizedAffineTransformation(self.transformations[i], frac)
		eAtT = Geometry.applyMatrixTransformToLineString(transformationAtT, e)
		return eAtT
