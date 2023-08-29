from typing import List

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion
from rt_bi_core.Model.SymbolRegion import SymbolRegion


class RegularSymbol(RegularSpatialRegion):
	"""A Class to model Regular Symbols."""
	def __init__(self, regions: List[SymbolRegion] = []):
		"""
		A dictionary from `region.name` to the `SymbolRegion` object.
		"""
		super().__init__(regions=regions)

	def addConnectedComponent(self, region: SymbolRegion) -> None:
		if region.regionType != SymbolRegion.RegionType.SENSING:
			RosUtils.Logger().info("Cannot add region %s to RegularSpatialRegion of type %s" % (repr(region.regionType), __class__.__name__))
			return
		return super().addConnectedComponent(region)
