from typing import List

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

	def addConnectedComponent(self, region: MapRegion) -> None:
		if region.regionType != MapRegion.RegionType.SENSING:
			RosUtils.Logger().info("Cannot add region %s to RegularSpatialRegion of type %s" % (repr(region.regionType), __class__.__name__))
			return
		return super().addConnectedComponent(region)
