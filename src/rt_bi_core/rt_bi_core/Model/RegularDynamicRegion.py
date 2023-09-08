from typing import List

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion


class RegularDynamicRegion(RegularSpatialRegion):
	"""A Class to model a set of Regular Dynamic Region."""
	def __init__(self, regions: List[DynamicRegion] = []):
		"""
		A dictionary from `sensor.name` to the `RegularDynamicRegion` object.
		"""
		super().__init__(regions=regions)

	def addConnectedComponent(self, region: DynamicRegion) -> None:
		if not self.isEmpty and abs(self.timeNanoSec - region.timeNanoSecs) > self.MAX_UPDATE_DELAY:
			RosUtils.Logger().warn("Discarded old region. Cannot add regions older than 0.5s to the same %s. self.t = %d and region.t = %d" % (self.__class__.__name__, self.timeNanoSec, region.timeNanoSecs))
		return super().addConnectedComponent(region)
