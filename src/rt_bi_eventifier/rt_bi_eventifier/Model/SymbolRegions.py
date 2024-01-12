from typing import List, Sequence, Set

from visualization_msgs.msg import Marker

from rt_bi_core.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.SymbolRegion import SymbolRegion


class SymbolRegions(RegularAffineRegion[SymbolRegion]):
	"""A Class to model Regular Symbols."""
	def __init__(self, regions: List[SymbolRegion] = []):
		"""
		A dictionary from `region.name` to the `SymbolRegion` object.
		"""
		super().__init__(regions=regions)

	def __and__(self, others: "SymbolRegions") -> Set[str]:
		return super().__and__(others)

	def __add__(self, others: "SymbolRegions") -> Set[str]:
		return super().__add__(others)

	def __sub__(self, others: "SymbolRegions") -> Set[str]:
		return super().__sub__(others)

	def __getitem__(self, regionName: str) -> SymbolRegion:
		return super().__getitem__(regionName)

	def getRegionsByShortName(self, shortName: str) -> List[SymbolRegion]:
		filteredNames = filter(lambda r: self[r].shortName == shortName, self)
		return [self[r] for r in filteredNames]

	def addConnectedComponent(self, region: SymbolRegion) -> None:
		if region.regionType != SymbolRegion.RegionType.SYMBOL: raise TypeError(f"Incorrect region type {region.regionType}")
		return super().addConnectedComponent(region)

	def intersection(self, others: "SymbolRegions") -> Set[str]:
		return super().intersection(others)

	def union(self, others: "SymbolRegions") -> Set[str]:
		return super().union(others)

	def difference(self, others: "SymbolRegions") -> Set[str]:
		return super().difference(others)

	def render(self) -> Sequence[Marker]:
		return super().render(durationNs=SymbolRegion.DEFAULT_RENDER_DURATION)
