from typing import List, Set

from rt_bi_core.Model.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion


class RegularSymbol(RegularAffineRegion):
	"""A Class to model Regular Symbols."""
	def __init__(self, regions: List[SymbolRegion] = []):
		"""
		A dictionary from `region.name` to the `SymbolRegion` object.
		"""
		super().__init__(regions=regions)

	def __and__(self, others: "RegularSymbol") -> Set[str]:
		return super().__and__(others)

	def __add__(self, others: "RegularSymbol") -> Set[str]:
		return super().__add__(others)

	def __sub__(self, others: "RegularSymbol") -> Set[str]:
		return super().__sub__(others)

	def __getitem__(self, regionName: str) -> SymbolRegion:
		return super().__getitem__(regionName)

	def addConnectedComponent(self, region: SymbolRegion) -> None:
		if region.regionType != SymbolRegion.RegionType.SYMBOL: return
		return super().addConnectedComponent(region)

	def intersection(self, others: "RegularSymbol") -> Set[str]:
		return super().intersection(others)

	def union(self, others: "RegularSymbol") -> Set[str]:
		return super().union(others)

	def difference(self, others: "RegularSymbol") -> Set[str]:
		return super().difference(others)
