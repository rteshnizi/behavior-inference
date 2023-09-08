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

	def __add__(self, other: "Shadows") -> "Shadows":
		return super().__add__(other)

	def __sub__(self, other: "Shadows") -> "Shadows":
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> ShadowRegion:
		return super().__getitem__(regionName)

	def addConnectedComponent(self, region: ShadowRegion) -> None:
		if region.regionType != ShadowRegion.RegionType.SHADOW:
			return
			RosUtils.Logger().info("Cannot add region %s to RegularSpatialRegion of type %s" % (repr(region.regionType), __class__.__name__))
		return super().addConnectedComponent(region)
