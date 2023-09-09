from typing import List

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion


class RegularDynamicRegion(RegularSpatialRegion):
	"""
	A Class to model a set of Regular Dynamic Region.
	Dynamic means, turns on, off, and/or undergoes an affine transformation.
	"""

	NANO_CONSTANT = 10 ** 9
	MAX_UPDATE_DELAY_NS = 5 * NANO_CONSTANT
	"""
	### Core assumption:
	We allow at most a delay of 5s between updates from a certain sensor before we declare it off.
	"""

	def __init__(self, regions: List[DynamicRegion] = []):
		super().__init__(regions=regions)

	def __getitem__(self, regionName: str) -> DynamicRegion:
		return super().__getitem__(regionName)

	@property
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		else: return self[next(iter(self))].timeNanoSecs

	def addConnectedComponent(self, region: DynamicRegion) -> None:
		if not self.isEmpty and self.timeNanoSec - region.timeNanoSecs > self.MAX_UPDATE_DELAY_NS:
			RosUtils.Logger().warn(
				"Discarded old region. Cannot add regions older than %.2fs to the same %s. self.t = %d and region.t = %d" %
				(self.MAX_UPDATE_DELAY_NS / self.NANO_CONSTANT, self.__class__.__name__, self.timeNanoSec, region.timeNanoSecs)
			)
		return super().addConnectedComponent(region)
