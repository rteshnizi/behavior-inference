from typing import List

from rt_bi_core.Model.RegularDynamicRegion import RegularDynamicRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion


class RegularSymbol(RegularDynamicRegion):
	"""A Class to model Regular Symbols."""
	def __init__(self, regions: List[SymbolRegion] = []):
		"""
		A dictionary from `region.name` to the `SymbolRegion` object.
		"""
		super().__init__(regions=regions)

	def __add__(self, other: "RegularSymbol") -> "RegularSymbol":
		return super().__add__(other)

	def __sub__(self, other: "RegularSymbol") -> "RegularSymbol":
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> SymbolRegion:
		return super().__getitem__(regionName)

	def addConnectedComponent(self, region: SymbolRegion) -> None:
		if region.regionType != SymbolRegion.RegionType.SYMBOL: return
		return super().addConnectedComponent(region)
