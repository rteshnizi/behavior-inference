from math import isnan, nan
from typing import Generic, Sequence, TypeAlias, TypeVar

from rt_bi_commons.Utils.Geometry import AffineTransform, GeometryLib, Shapely
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon
from rt_bi_core.Spatial.MovingPolygon import MovingPolygon
from rt_bi_core.Spatial.SensingPolygon import SensingPolygon
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon
from rt_bi_core.Spatial.Tracklet import Tracklet

_InputPolyTypes: TypeAlias = MovingPolygon | StaticPolygon | SensingPolygon
_T_Poly = TypeVar("_T_Poly", bound=_InputPolyTypes)
class ContinuousTimeRegion(Generic[_T_Poly]):
	def __init__(self, polyConfigs: Sequence[_T_Poly]) -> None:
		self.__sortedConfigs: list[_T_Poly] = list(sorted(polyConfigs, key=lambda r: r.timeNanoSecs))
		self.__transforms: Sequence[AffineTransform] = self.__initTransformationMatrices()
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
		if self.type == SensingPolygon.Types.SENSING:
			tracklets: Dict[int, Tracklet] = self.configs[i - 1].tracklets # type: ignore
			region = SensingPolygon(centerOfRotation=cor, id=self.id, envelope=[], timeNanoSecs=timeNanoSecs, interior=regionPoly, tracklets=tracklets)
		elif self.configs[i - 1].type == StaticPolygon.Types.STATIC:
			region = StaticPolygon(self.configs[i - 1].id.polygonId, self.configs[i - 1].id.regionId, envelope=[], timeNanoSecs=timeNanoSecs, interior=regionPoly)
		elif self.configs[i - 1].type == AffinePolygon.Types.DYNAMIC:
			region = AffinePolygon(self.configs[i - 1].centerOfRotation, self.id, [], timeNanoSecs, overlappingPolygonId=self.configs[i - 1].overlappingRegionId, overlappingRegionType=self.configs[i - 1].inFov, interior=regionPoly) # type: ignore
		else:
			raise TypeError("Unexpected region type: %s" % repr(self.type))
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
		return f"{self.type}" if self.length == 0 else self.configs[0].shortName

	@property
	def id(self) -> MovingPolygon.Id:
		if self.length == 0: return MovingPolygon.Id("", "", "", "")
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

	def getEdgeBb(self, eName: str) -> Shapely.Polygon | Shapely.LineString:
		"""Get Edge Bounding Box

		:param str eName: edge name
		:return: The tightest bounding box around the given line segment.
		:rtype: `Shapely.Polygon` or `Shapely.LineString`
		"""
		edge = self.configs[0].edges[eName]
		transform = self.__getOverallTransformationMatrix()
		return GeometryLib.getLineSegmentExpandedBb(transform, edge, self.configs[0].centerOfRotation)

	def getEdgeAt(self, eName: str, timeNanoSecs: int) -> Shapely.LineString:
		e = self.configs[0].edges[eName]
		i = self.__timeNanoSecsToIndex(timeNanoSecs)
		frac = self.__getParameterizedTime(i, timeNanoSecs)
		transformationAtT = GeometryLib.getParameterizedAffineTransformation(self.transformations[i], frac)
		eAtT = GeometryLib.applyMatrixTransformToLineString(transformationAtT, e)
		return eAtT

	@staticmethod
	def fromMergedList(mergedList: list[_T_Poly]) -> "list[ContinuousTimeRegion[_T_Poly]]":
		byPolyId: dict[MovingPolygon.Id, list[_T_Poly]] = {}
		ctrList: list[ContinuousTimeRegion[_T_Poly]] = []
		for poly in mergedList:
			if poly.id not in byPolyId: byPolyId[poly.id] = [poly]
			else: byPolyId[poly.id].append(poly)
		for id in byPolyId:
			ctrList.append(ContinuousTimeRegion[_T_Poly](byPolyId[id]))
		return ctrList
