from typing import Generic, Iterator, Sequence, TypeVar

from visualization_msgs.msg import Marker

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.Geometry import GeometryLib, Shapely
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.Polygons.Polygon import Polygon

_RegionType = TypeVar("_RegionType", bound=Polygon)


class SpatialRegion(Generic[_RegionType]):
	"""A Regular Spatial Region. Polygons may be static or dynamic."""

	def __init__(self, regions: Sequence[_RegionType] = []):
		self.__regions: dict[Polygon.Id, _RegionType] = {}
		"""
		A dictionary from `region.name` to the `RegionType` object.
		"""
		for region in regions: self.addConnectedComponent(region)

	def __and__(self, others: "SpatialRegion") -> set[Polygon.Id]:
		regions = set(self.__regions.keys())
		otherRegions = set(others.__regions.keys())
		intersection = regions & otherRegions
		return intersection

	def __sub__(self, others: "SpatialRegion") -> set[Polygon.Id]:
		selfRegions = set(self.__regions.keys())
		otherRegions = set(others.__regions.keys())
		diff = selfRegions - otherRegions
		return diff

	def __add__(self, others: "SpatialRegion") -> set[Polygon.Id]:
		selfRegions = set(self.__regions.keys())
		otherRegions = set(others.__regions.keys())
		union = selfRegions.union(otherRegions)
		return union

	def __iter__(self) -> Iterator[Polygon.Id]:
		return iter(self.__regions)

	def __contains__(self, regionName: str) -> bool:
		return regionName in self.__regions

	def __next__(self) -> Polygon.Id:
		return next(iter(self))

	def __getitem__(self, polyId: Polygon.Id) -> _RegionType:
		if not isinstance(polyId, Polygon.Id):
			raise AssertionError(f"Regular regions are dictionaries. Index must be a `Polygon.Id`.")
		return self.__regions[polyId]

	def __len__(self) -> int:
		return len(self.__regions)

	def __repr__(self):
		return "%s" % repr([i for i in self.__regions])

	@property
	def envelopePolygon(self) -> Shapely.Polygon | Shapely.MultiPolygon:
		"""The geometrical representation of the envelope of the union of the regions."""
		polygons = [Shapely.Polygon(self.__regions[name].envelope) for name in self.__regions]
		polygon = GeometryLib.union(polygons)
		return polygon

	@property
	def regionIds(self) -> list[Polygon.Id]:
		"""List of the names of the regions, which also happens to be the list of the nodes of the graph as well."""
		return list(self.__regions.keys())

	@property
	def interior(self) -> Shapely.Polygon | Shapely.MultiPolygon:
		"""The geometrical representation of the FOV of the union of the regions."""
		polygons = [self.__regions[name].interior for name in self.__regions]
		polygon = GeometryLib.union(polygons)
		return polygon

	@property
	def isEmpty(self) -> bool:
		return len(self) == 0

	@property
	def edges(self) -> dict[str, Shapely.LineString]:
		raise NotImplementedError()

	@property
	def regionType(self) -> Polygon.Types:
		for rName in self: return self[rName].type
		return Polygon.Types.BASE

	def addConnectedComponent(self, region: _RegionType) -> None:
		if region.name in self.__regions:
			Ros.Logger().debug("Overriding region with name %s that is already added in %s." % (region.name, self.__class__.__name__))
		self.__regions[region.id] = region
		return

	def intersection(self, others: "SpatialRegion") -> set[Polygon.Id]:
		return self.__and__(others)

	def union(self, others: "SpatialRegion") -> set[Polygon.Id]:
		return self.__add__(others)

	def difference(self, others: "SpatialRegion") -> set[Polygon.Id]:
		return self.__sub__(others)

	def render(self, durationNs: int = Polygon.DEFAULT_RENDER_DURATION_NS, envelopeColor: RGBA | None = None) -> list[Marker]:
		markers = []
		for region in self.__regions.values():
			Ros.ConcatMessageArray(markers, region.render(envelopeColor=envelopeColor, durationNs=durationNs))
		return markers

	def clearRender(self) -> list[Marker]:
		markers = []
		for region in self.__regions.values():
			Ros.ConcatMessageArray(markers, region.clearRender())
		return markers
