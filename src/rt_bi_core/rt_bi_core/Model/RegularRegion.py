from typing import Dict, List, Type, Union

from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry, LineString, MultiPolygon, Polygon


class RegularSpatialRegion:
	"""The base class for all regular spatial regions."""
	def __init__(self, regions: List[Type[PolygonalRegion]] = []):
		self.__regions: Dict[str, Type[PolygonalRegion]] = {}
		"""
		A dictionary from `region.name` to the `PolygonalRegion` object.
		For subclasses, the region types depend on the subclass.
		"""
		for region in regions: self.addConnectedComponent(region)

	def __iter__(self):
		return iter(self.__regions)

	def __next__(self) -> str:
		return next(self.__regions)

	def __getitem__(self, regionName: str) -> Type[PolygonalRegion]:
		return self.__regions[regionName]

	def __repr__(self):
		return "%s" % repr([self.__regions.values()])

	@property
	def envelopePolygon(self) -> Union[Polygon, MultiPolygon]:
		"""The geometrical representation of the envelope of the union of the regions."""
		polygons = [Polygon(self.__regions[name].envelope) for name in self.__regions]
		polygon = Geometry.union(polygons)
		return polygon

	@property
	def regionNames(self) -> List[str]:
		"""List of the names of the regions, which also happens to be the list of the nodes of the graph as well."""
		return ["not", "implemented", "lol", "!"]

	@property
	def interior(self) -> Union[Polygon, MultiPolygon]:
		"""The geometrical representation of the FOV of the union of the regions."""
		polygons = [self.__regions[name].fov for name in self.__regions]
		polygon = Geometry.union(polygons)
		return polygon

	@property
	def timeNanoSec(self) -> int:
		return 0

	@property
	def edges(self) -> Dict[str, LineString]:
		return self.__edges

	def addConnectedComponent(self, region: PolygonalRegion) -> None:
		if self.timeNanoSec != region.timeNanoSecs:
			raise RuntimeError("Cannot add Regular Regions at different times to the same object.")
		if region.name in self.__regions:
			raise RuntimeError("Region with name %s is already added to %s." % (region.name, self.__class__.__name__))
		self.__regions[region.name] = region
		return
