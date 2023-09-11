from typing import List, Set

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion


class MapPerimeter(RegularSpatialRegion):
	"""A Class to model Shadows."""
	def __init__(self, regions: List[MapRegion] = []):
		"""
		A dictionary from `region.name` to the `MapRegion` object.
		"""
		super().__init__(regions=regions)

	def __and__(self, other: "MapPerimeter") -> Set[str]:
		return super().__and__(other)

	def __add__(self, other: "MapPerimeter") -> Set[str]:
		return super().__add__(other)

	def __sub__(self, other: "MapPerimeter") -> Set[str]:
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> MapRegion:
		return super().__getitem__(regionName)

	def addConnectedComponent(self, region: MapRegion) -> None:
		if region.regionType != MapRegion.RegionType.MAP: return
		return super().addConnectedComponent(region)

	def intersection(self, other: "MapPerimeter") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "MapPerimeter") -> Set[str]:
		return super().union(other)

	def difference(self, other: "MapPerimeter") -> Set[str]:
		return super().difference(other)
