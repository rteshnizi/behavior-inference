from math import isnan, nan
from typing import Any, Final, Generic, TypeVar, cast

from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_core.Spatial import GraphPolygon, PolygonFactory
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.Polygon import PolygonFactoryKeys
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon

_T_Poly = TypeVar("_T_Poly", bound=GraphPolygon)
class ContinuousTimeRegion(Generic[_T_Poly]):
	INF_NS: Final[int] = 2 ** 100
	__MAX_HISTORY = 2
	def __init__(self, polyConfigs: list[_T_Poly]) -> None:
		self.__sortedConfigs: list[_T_Poly] = []
		for poly in polyConfigs: self.addPolygon(poly, -1)
		return

	def __repr__(self) -> str:
		tStr = f"{self.earliestNanoSecs}, {self.latestNanoSecs}"
		if self.isStatic or self.earliestNanoSecs == self.latestNanoSecs:
			tStr = f"{self.earliestNanoSecs}"
		return f"CTR-{self.name}[{tStr}]"

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
		if self.earliestNanoSecs == -1: raise ValueError(f"earliestNanoSecs is not set in {self.name}.")
		if self.latestNanoSecs == -1: raise ValueError(f"latestNanoSecs is not set in {self.name}.")

		if self.isStatic: return self.configs[0]

		if timeNanoSecs < self.earliestNanoSecs: raise IndexError(f"{timeNanoSecs} is less than earliestNanoSecs={self.earliestNanoSecs} in {self.name}.")
		if timeNanoSecs > self.latestNanoSecs: raise IndexError(f"{timeNanoSecs} is greater than latestNanoSecs={self.latestNanoSecs} in {self.name}.")
		if timeNanoSecs == self.earliestNanoSecs: return self.configs[0]
		if timeNanoSecs == self.latestNanoSecs: return self.configs[-1]
		i = 0
		for i in range(self.length):
			if timeNanoSecs <= self.configs[i].timeNanoSecs: break
		param = self.__getParameterizedTime(i - 1, timeNanoSecs)
		if isnan(param):
			return self.configs[i - 1]
		transform = GeometryLib.getParameterizedAffineTransformation(self.__transformation(), param)
		cor = GeometryLib.applyMatrixTransformToCoords(transform, self.configs[i - 1].centerOfRotation)
		poly = GeometryLib.applyMatrixTransformToPolygon(transform, self.configs[i - 1].interior)
		kwArgs: dict[PolygonFactoryKeys, Any] = {
			"polygonId": self.id.polygonId,
			"regionId": self.id.regionId,
			"subPartId": self.id.subPartId,
			"centerOfRotation": cor,
			"envelope": [],
			"timeNanoSecs": timeNanoSecs,
			"interior": poly,
			"hIndex": -1,
			"predicates": self.predicates,
		}
		if self.type == SensingPolygon.type:
			kwArgs["tracklets"] = cast(SensingPolygon, self.configs[i - 1]).tracklets
			Cls = SensingPolygon
		elif self.isStatic:
			kwArgs.pop("centerOfRotation", None)
			kwArgs["timeNanoSecs"] = self.configs[i - 1].timeNanoSecs
			Cls = StaticPolygon
		elif self.type == MovingPolygon.type:
			Cls = MovingPolygon
		else:
			raise AssertionError(f"Unexpected region type: {repr(self.type)} in {self.name}")
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
		if self.length == 0: return self.type.value
		(regionId, polyId) = self.configs[0].id.shortNames()
		name = f"{self.type.value}-{regionId}-{polyId}"
		return name

	@property
	def id(self) -> MovingPolygon.Id:
		if self.length == 0: return MovingPolygon.Id(hIndex=-1, timeNanoSecs=-1, regionId="", polygonId="", subPartId="")
		return self.configs[0].id

	@property
	def length(self) -> int:
		return len(self.__sortedConfigs)

	@property
	def configs(self) -> list[_T_Poly]:
		return self.__sortedConfigs

	@property
	def type(self) -> MovingPolygon.Types:
		if self.length == 0: return MovingPolygon.Types.BASE
		return self.configs[0].type

	@property
	def latestNanoSecs(self) -> int:
		if self.length == 0: return -1
		if self.isStatic: return self.INF_NS
		return self.__sortedConfigs[-1].timeNanoSecs

	@property
	def earliestNanoSecs(self) -> int:
		if self.length == 0: return -1
		return self.__sortedConfigs[0].timeNanoSecs

	@property
	def isSlice(self) -> bool:
		return self.earliestNanoSecs == self.latestNanoSecs and self.earliestNanoSecs != -1

	@property
	def isStatic(self) -> bool:
		return self.type == StaticPolygon.type

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

	def __transformation(self) -> AffineTransform:
		return GeometryLib.getAffineTransformation(self.configs[0].envelope, self.configs[-1].envelope)

	def __transformationAt(self, timeNanoSecs: int) -> AffineTransform:
		assert timeNanoSecs in self, f"Requested time {timeNanoSecs} is out of range: {self.name} -- {self.earliestNanoSecs}-{self.latestNanoSecs}"
		if self.length == 1: return GeometryLib.getAffineTransformation(None, None)

		frac = self.__getParameterizedTime(0, timeNanoSecs)
		transformationAtT = GeometryLib.getParameterizedAffineTransformation(self.__transformation(), frac)
		return transformationAtT

	def getEdgeBb(self, edge: Shapely.LineString, upToNs: int) -> Shapely.Polygon | Shapely.LineString:
		"""Get Edge Bounding Box

		:param str eName: edge name
		:return: The tightest bounding box around the given line segment.
		:rtype: `Shapely.Polygon` or `Shapely.LineString`
		"""
		if self.isStatic: return edge
		transform = self.__transformationAt(upToNs)
		return GeometryLib.getLineSegmentExpandedBb(transform, edge, self.configs[0].centerOfRotation)

	def getEdgeAt(self, edge: Shapely.LineString, timeNanoSecs: int) -> Shapely.LineString:
		if self.isStatic or self.length == 1: return edge
		transform = self.__transformationAt(timeNanoSecs)
		return GeometryLib.applyMatrixTransformToLineString(transform, edge)

	def addPolygon(self, polygon: _T_Poly, keepFromNs: int) -> None:
		if self.length > 0:
			assert self.id.sansTime() == polygon.id.sansTime(), (
				f"Different ids in poly configs. {self.id} vs {polygon.id}. " +
				"A ContinuousTimeRegion must describe the evolution of a single polygon."
			)
		self.__sortedConfigs.append(polygon)
		self.__sortedConfigs = sorted(self.__sortedConfigs, key=lambda p: p.timeNanoSecs)

		if len(self.__sortedConfigs) > self.__MAX_HISTORY:
			popIndex = 1
			if self.__sortedConfigs[1].timeNanoSecs <= keepFromNs: popIndex = 0
			c = self.__sortedConfigs.pop(popIndex)
			Ros.Log(f"Dropping from {self.name}: {c.timeNanoSecs}")
		return
