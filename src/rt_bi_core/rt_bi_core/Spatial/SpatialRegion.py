from typing import Generic, Iterator, Sequence, TypeVar

from visualization_msgs.msg import Marker

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.Spatial.Polygon import Polygon

_T_Poly = TypeVar("_T_Poly", bound=Polygon)


class SpatialRegion(Generic[_T_Poly]):
	"""A Regular Spatial Region. Polygons may be static or dynamic."""

	MAX_UPDATE_DELAY_S = 15
	"""
	We allow at most a delay of some noticeable seconds between updates from a certain sensor before we declare it off.
	"""

	MAX_UPDATE_DELAY_NS = MAX_UPDATE_DELAY_S * Polygon.NANO_CONVERSION_CONSTANT
	"""
	We allow at most a delay of some noticeable seconds between updates from a certain sensor before we declare it off.
	"""

	def __init__(self, regions: Sequence[_T_Poly] = []):
		self.__polygons: dict[Polygon.Id, _T_Poly] = {}
		"""
		A dictionary from `region.name` to the `RegionType` object.
		"""
		for region in regions: self.addConnectedComponent(region)

	def __and__(self, others: "SpatialRegion") -> set[Polygon.Id]:
		regions = set(self.__polygons.keys())
		otherRegions = set(others.__polygons.keys())
		intersection = regions & otherRegions
		return intersection

	def __sub__(self, others: "SpatialRegion") -> set[Polygon.Id]:
		selfRegions = set(self.__polygons.keys())
		otherRegions = set(others.__polygons.keys())
		diff = selfRegions - otherRegions
		return diff

	def __add__(self, others: "SpatialRegion") -> set[Polygon.Id]:
		selfRegions = set(self.__polygons.keys())
		otherRegions = set(others.__polygons.keys())
		union = selfRegions.union(otherRegions)
		return union

	def __iter__(self) -> Iterator[Polygon.Id]:
		return iter(self.__polygons)

	def __contains__(self, id: Polygon.Id) -> bool:
		return id in self.__polygons

	def __next__(self) -> Polygon.Id:
		return next(iter(self))

	def __setitem__(self, polyId: Polygon.Id, value: _T_Poly) -> None:
		if not isinstance(polyId, Polygon.Id):
			raise AssertionError("Regular regions are dictionaries. Index must be a `Polygon.Id`.")
		self.__polygons[polyId] = value
		return

	def __getitem__(self, polyId: Polygon.Id) -> _T_Poly:
		if not isinstance(polyId, Polygon.Id):
			raise AssertionError("Regular regions are dictionaries. Index must be a `Polygon.Id`.")
		return self.__polygons[polyId]

	def __len__(self) -> int:
		return len(self.__polygons)

	def __repr__(self):
		return "%s" % repr([i for i in self.__polygons])

	@property
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		return self[self.polygonIds[0]].timeNanoSecs

	@timeNanoSec.setter
	def timeNanoSec(self, t: int) -> None:
		for rName in self.polygonIds: self[rName].timeNanoSecs = t
		return

	@property
	def envelopePolygon(self) -> Shapely.Polygon | Shapely.MultiPolygon:
		"""The geometrical representation of the envelope of the union of the regions."""
		polygons = [Shapely.Polygon(self.__polygons[name].envelope) for name in self.__polygons]
		polygon = GeometryLib.union(polygons)
		return polygon

	@property
	def polygons(self) -> list[_T_Poly]:
		return list(self.__polygons.values())

	@property
	def polygonIds(self) -> list[Polygon.Id]:
		"""List of the ids of the polygons, which also happens to be the list of the nodes of the graph as well."""
		return list(self.__polygons.keys())

	@property
	def interior(self) -> Shapely.Polygon | Shapely.MultiPolygon:
		"""The geometrical representation of the FOV of the union of the regions."""
		polygons = [self.__polygons[name].interior for name in self.__polygons]
		polygon = GeometryLib.union(polygons)
		return polygon

	@property
	def isEmpty(self) -> bool:
		return len(self) == 0

	@property
	def edges(self) -> dict[str, Shapely.LineString]:
		raise NotImplementedError()

	@property
	def type(self) -> Polygon.Types:
		for rName in self: return self[rName].type
		return Polygon.Types.BASE

	def getAllParts(self, id: Polygon.Id) -> list[_T_Poly]:
		"""
		Find all polygons in this regular region which share the same src polygon.
		This is useful in finding split parts of a moving region.
		"""
		matches = list(filter(lambda x: (x.id.polygonId == id.polygonId and x.id.regionId == id.regionId), self.polygons))
		return matches

	def addConnectedComponent(self, region: _T_Poly) -> None:
		deltaT = self.timeNanoSec - region.timeNanoSecs
		if not self.isEmpty and deltaT > self.MAX_UPDATE_DELAY_NS:
			Ros.Logger().debug(
				f"Discarded old region {repr(region)}. Given region is older than {self.MAX_UPDATE_DELAY_S}s. Time difference = {deltaT} ns."
			)
			return
		if region.id in self.polygonIds and self.timeNanoSec > region.timeNanoSecs:
			Ros.Logger().debug(
				f"Region {repr(region)} already exist and will not be replaced by older version. self.t = {self.timeNanoSec} and region.t = {region.timeNanoSecs}"
			)
			return
		if region.id in self:
			Ros.Logger().debug(f"Overriding region {region.shortName}.")
		self.__polygons[region.id] = region
		return

	def intersection(self, others: "SpatialRegion") -> set[Polygon.Id]:
		return self.__and__(others)

	def union(self, others: "SpatialRegion") -> set[Polygon.Id]:
		return self.__add__(others)

	def difference(self, others: "SpatialRegion") -> set[Polygon.Id]:
		return self.__sub__(others)

	def render(self, durationNs: int = Polygon.DEFAULT_RENDER_DURATION_NS, envelopeColor: RGBA | None = None) -> list[Marker]:
		markers = []
		for region in self.__polygons.values():
			Ros.ConcatMessageArray(markers, region.render(envelopeColor=envelopeColor, durationNs=durationNs))
		return markers

	def clearRender(self) -> list[Marker]:
		markers = []
		for region in self.__polygons.values():
			Ros.ConcatMessageArray(markers, region.clearRender())
		return markers
