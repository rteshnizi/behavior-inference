from math import isnan, nan
from typing import Any, Generic, TypeVar, cast

from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_core.Spatial import GraphInputPolygon, PolygonFactory
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.Polygon import PolygonFactoryKeys
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon

_T_Poly = TypeVar("_T_Poly", bound=GraphInputPolygon)
class ContinuousTimeRegion(Generic[_T_Poly]):
	def __init__(self, polyConfigs: list[_T_Poly]) -> None:
		self.__sortedConfigs: list[_T_Poly] = list(sorted(polyConfigs, key=lambda r: r.timeNanoSecs))
		self.__transforms: list[AffineTransform] = self.__initTransformationMatrices()
		return

	def __repr__(self) -> str:
		return "CTR-%s[%d, %d]" % (self.name, self.earliestNanoSecs, self.latestNanoSecs)

	def __getitem__(self, timeNanoSecs: int) -> _T_Poly:
		"""Get polygonal shape at time.

		Parameters
		----------
		timeNanoSecs : int
			Time in NanoSeconds.

		Returns
		-------
		Shapely.Polygon
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
		transform = GeometryLib.getParameterizedAffineTransformation(self.__transforms[i - 1], param)
		cor = GeometryLib.applyMatrixTransformToCoords(transform, self.configs[i - 1].centerOfRotation)
		regionPoly = GeometryLib.applyMatrixTransformToPolygon(transform, self.configs[i - 1].interior)
		kwArgs: dict[PolygonFactoryKeys, Any] = {
			"polygonId": self.id.polygonId,
			"regionId": self.id.regionId,
			"centerOfRotation": cor,
			"envelope": [],
			"timeNanoSecs": timeNanoSecs,
			"interior": regionPoly,
			"predicates": self.predicates,
		}
		if self.type == SensingPolygon.type:
			kwArgs["tracklets"] = cast(SensingPolygon, self.configs[i - 1]).tracklets
			Cls = SensingPolygon
		elif self.type == StaticPolygon.type:
			kwArgs.pop("centerOfRotation", None)
			Cls = StaticPolygon
		elif self.type == MovingPolygon.type:
			Cls = MovingPolygon
		else:
			raise AssertionError(f"Unexpected region type: {repr(self.type)}")
		poly = PolygonFactory(Cls, kwArgs)
		return cast(_T_Poly, poly)

	def __contains__(self, timeNanoSecs: int) -> bool:
		if self.earliestNanoSecs == -1: return False
		if self.latestNanoSecs == -1: return False
		if timeNanoSecs < self.earliestNanoSecs: return False
		if timeNanoSecs > self.latestNanoSecs: return False
		return True

	@property
	def predicates(self) -> Predicates:
		if self.length == 0: return Predicates([])
		return self.configs[0].predicates

	@property
	def name(self) -> str:
		"""Region name."""
		return f"{self.type}" if self.length == 0 else self.configs[0].shortName

	@property
	def id(self) -> MovingPolygon.Id:
		if self.length == 0: return MovingPolygon.Id(timeNanoSecs=-1, regionId="", polygonId="")
		return self.configs[0].id

	@property
	def length(self) -> int:
		return len(self.__sortedConfigs)

	@property
	def configs(self) -> list[_T_Poly]:
		return self.__sortedConfigs

	@property
	def type(self) -> MovingPolygon.Types:
		if self.length == 0: return MovingPolygon.type
		return self.configs[0].type

	@property
	def transformations(self) -> list[AffineTransform]:
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

	def __initTransformationMatrices(self) -> list[AffineTransform]:
		transforms: list[AffineTransform] = []
		if self.length < 2: return transforms
		for i in range(self.length - 1):
			if self.configs[i].id != self.configs[i + 1].id:
				raise AssertionError(f"Different ids in poly configs. {self.configs[i].id} vs {self.configs[i + 1].id} A ContinuousTimeRegion must describe the evolution of a single polygon.")
			transform = GeometryLib.getAffineTransformation(self.configs[i].envelope, self.configs[i + 1].envelope)
			transforms.append(transform)
		return transforms

	def __getOverallTransformationMatrix(self) -> AffineTransform:
		if self.length < 2: return AffineTransform()
		transform = GeometryLib.getAffineTransformation(self.configs[0].envelope, self.configs[-1].envelope)
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

	def getEdgeBb(self, edge: Shapely.LineString) -> Shapely.Polygon | Shapely.LineString:
		"""Get Edge Bounding Box

		:param str eName: edge name
		:return: The tightest bounding box around the given line segment.
		:rtype: `Shapely.Polygon` or `Shapely.LineString`
		"""
		transform = self.__getOverallTransformationMatrix()
		return GeometryLib.getLineSegmentExpandedBb(transform, edge, self.configs[0].centerOfRotation)

	def getEdgeAt(self, edge: Shapely.LineString, timeNanoSecs: int) -> Shapely.LineString:
		i = self.__timeNanoSecsToIndex(timeNanoSecs)
		frac = self.__getParameterizedTime(i, timeNanoSecs)
		transformationAtT = GeometryLib.getParameterizedAffineTransformation(self.transformations[i], frac)
		eAtT = GeometryLib.applyMatrixTransformToLineString(transformationAtT, edge)
		return eAtT

	@staticmethod
	def fromMergedList(mergedList: list[_T_Poly]) -> "list[ContinuousTimeRegion[_T_Poly]]":
		byPolyId: dict[MovingPolygon.Id, list[_T_Poly]] = {}
		ctrList: list[ContinuousTimeRegion[_T_Poly]] = []
		for poly in mergedList:
			timeLessId = poly.id.updateTime(-1)
			if poly.id not in byPolyId: byPolyId[timeLessId] = [poly]
			else: byPolyId[timeLessId].append(poly)
		for id in byPolyId:
			ctrList.append(ContinuousTimeRegion[_T_Poly](byPolyId[id]))
		return ctrList
