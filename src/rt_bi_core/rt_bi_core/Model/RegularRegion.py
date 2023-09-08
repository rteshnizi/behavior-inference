from typing import Dict, List, Set, Type, Union

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry, LineString, MultiPolygon, Polygon


class RegularSpatialRegion:
	"""The base class for all regular spatial regions."""

	MAX_UPDATE_DELAY = 5 ** 8
	"""
	### Core assumption:
	We allow at most a delay of 0.5s between updates from a certain sensor before we declare it off.
	"""

	def __init__(self, regions: List[Type[PolygonalRegion]] = []):
		self.__regions: Dict[str, Type[PolygonalRegion]] = {}
		"""
		A dictionary from `region.name` to the `PolygonalRegion` object.
		For subclasses, the region types depend on the subclass.
		"""
		for region in regions: self.addConnectedComponent(region)

	def __and__(self, other: "RegularSpatialRegion") -> Set[str]:
		regions = set(self.__regions.keys())
		otherRegions = set(other.__regions.keys())
		intersection = regions & otherRegions
		return intersection

	def __sub__(self, other: "RegularSpatialRegion") -> Set[str]:
		selfRegions = set(self.__regions.keys())
		otherRegions = set(other.__regions.keys())
		diff = selfRegions - otherRegions
		return diff

	def __add__(self, other: "RegularSpatialRegion") -> Set[str]:
		selfRegions = set(self.__regions.keys())
		otherRegions = set(other.__regions.keys())
		union = selfRegions.union(otherRegions)
		return union

	def __iter__(self):
		return iter(self.__regions)

	def __next__(self) -> str:
		return next(self.__regions)

	def __getitem__(self, regionName: str) -> Type[PolygonalRegion]:
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
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		else: return self[next(iter(self.__regions))].timeNanoSecs

	@property
	def isEmpty(self) -> bool:
		return len(self) == 0

	@property
	def edges(self) -> Dict[str, LineString]:
		return self.__edges

	def addConnectedComponent(self, region: Type[PolygonalRegion]) -> None:
		if region.name in self.__regions:
			RosUtils.Logger().warn("Overriding region with name %s that is already added in %s." % (region.name, self.__class__.__name__))
		self.__regions[region.name] = region
		return

	def intersection(self, other: "RegularSpatialRegion") -> Set[str]:
		return self.__and__(other)

	def union(self, other: "RegularSpatialRegion") -> Set[str]:
		return self.__add__(other)

	def difference(self, other: "RegularSpatialRegion") -> Set[str]:
		return self.__sub__(other)
