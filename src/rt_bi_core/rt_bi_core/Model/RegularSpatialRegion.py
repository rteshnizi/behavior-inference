from typing import Dict, Iterator, List, Sequence, Set, Type, Union

from visualization_msgs.msg import Marker

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry, LineString, MultiPolygon, Polygon
from rt_bi_utils.RViz import Color, RViz


class RegularSpatialRegion:
	"""The base class for all regular spatial regions."""

	def __init__(self, regions: Sequence[PolygonalRegion] = []):
		self.__regions: Dict[str, PolygonalRegion] = {}
		"""
		A dictionary from `region.name` to the `PolygonalRegion` object.
		For subclasses, the region types depend on the subclass.
		"""
		for region in regions: self.addConnectedComponent(region)

	def __and__(self, others: "RegularSpatialRegion") -> Set[str]:
		regions = set(self.__regions.keys())
		otherRegions = set(others.__regions.keys())
		intersection = regions & otherRegions
		return intersection

	def __sub__(self, others: "RegularSpatialRegion") -> Set[str]:
		selfRegions = set(self.__regions.keys())
		otherRegions = set(others.__regions.keys())
		diff = selfRegions - otherRegions
		return diff

	def __add__(self, others: "RegularSpatialRegion") -> Set[str]:
		selfRegions = set(self.__regions.keys())
		otherRegions = set(others.__regions.keys())
		union = selfRegions.union(otherRegions)
		return union

	def __iter__(self) -> Iterator[str]:
		return iter(self.__regions)

	def __next__(self) -> str:
		return next(iter(self))

	def __getitem__(self, regionName: str) -> PolygonalRegion:
		if not isinstance(regionName, str):
			RosUtils.Logger().error("Regular regions are dictionaries. Index must be string. Given %s" % repr(regionName))
			raise KeyError("Regular regions are dictionaries. Index must be string. Given %s" % repr(regionName))
		return self.__regions[regionName]

	def __len__(self) -> int:
		return len(self.__regions)

	def __repr__(self):
		return "%s" % repr([i for i in self.__regions])

	@property
	def envelopePolygon(self) -> Union[Polygon, MultiPolygon]:
		"""The geometrical representation of the envelope of the union of the regions."""
		polygons = [Polygon(self.__regions[name].envelope) for name in self.__regions]
		polygon = Geometry.union(polygons)
		return polygon

	@property
	def regionNames(self) -> List[str]:
		"""List of the names of the regions, which also happens to be the list of the nodes of the graph as well."""
		return list(self.__regions.keys())

	@property
	def interior(self) -> Union[Polygon, MultiPolygon]:
		"""The geometrical representation of the FOV of the union of the regions."""
		polygons = [self.__regions[name].interior for name in self.__regions]
		polygon = Geometry.union(polygons)
		return polygon

	@property
	def isEmpty(self) -> bool:
		return len(self) == 0

	@property
	def edges(self) -> Dict[str, LineString]:
		raise NotImplementedError()

	def addConnectedComponent(self, region: PolygonalRegion) -> None:
		if region.name in self.__regions:
			RosUtils.Logger().debug("Overriding region with name %s that is already added in %s." % (region.name, self.__class__.__name__))
		self.__regions[region.name] = region
		return

	def intersection(self, others: "RegularSpatialRegion") -> Set[str]:
		return self.__and__(others)

	def union(self, others: "RegularSpatialRegion") -> Set[str]:
		return self.__add__(others)

	def difference(self, others: "RegularSpatialRegion") -> Set[str]:
		return self.__sub__(others)

	def render(self, envelopeColor: Union[Color, None] = None) -> List[Marker]:
		markers = []
		for region in self.__regions.values():
			markers += region.render(envelopeColor=envelopeColor)
		return markers
