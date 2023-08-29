from typing import List

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion
from rt_bi_core.Model.ShadowRegion import ShadowRegion


class Shadows(RegularSpatialRegion):
	"""A Class to model Shadows."""
	def __init__(self, regions: List[ShadowRegion] = []):
		"""
		A dictionary from `region.name` to the `ShadowRegion` object.
		"""
		super().__init__(regions=regions)

	def addConnectedComponent(self, region: ShadowRegion) -> None:
		if region.regionType != ShadowRegion.RegionType.SENSING:
			return
			RosUtils.Logger().info("Cannot add region %s to RegularSpatialRegion of type %s" % (repr(region.regionType), __class__.__name__))
		return super().addConnectedComponent(region)
