from typing import Generic, Sequence, Set, TypeVar, Union

from visualization_msgs.msg import Marker

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_core.Model.RegularSpatialRegion import RegularSpatialRegion
from rt_bi_utils.RViz import RGBA

RegionType = TypeVar("RegionType", bound=DynamicRegion)

class RegularDynamicRegion(Generic[RegionType], RegularSpatialRegion[RegionType]):
	"""
	A Class to model a set of Regular Dynamic Region.
	Dynamic means, turns on, off, and/or undergoes an affine transformation.
	"""

	MAX_UPDATE_DELAY_NS = 15 * DynamicRegion.NANO_CONSTANT
	"""
	### Core assumption:
	We allow at most a delay of some noticeable seconds between updates from a certain sensor before we declare it off.
	"""

	def __init__(self, regions: Sequence[RegionType] = []):
		super().__init__(regions=regions)

	def __and__(self, other: "RegularDynamicRegion") -> Set[str]:
		return super().__and__(other)

	def __add__(self, other: "RegularDynamicRegion") -> Set[str]:
		return super().__add__(other)

	def __sub__(self, other: "RegularDynamicRegion") -> Set[str]:
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> RegionType:
		return super().__getitem__(regionName)

	@property
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		else: return self[next(iter(self))].timeNanoSecs

	def addConnectedComponent(self, region: RegionType) -> None:
		if not self.isEmpty and self.timeNanoSec - region.timeNanoSecs > self.MAX_UPDATE_DELAY_NS:
			RosUtils.Logger().debug(
				"Discarded old region %s. Given region is older than %fs. Time difference = %d ns." %
				(repr(region), self.MAX_UPDATE_DELAY_NS / DynamicRegion.NANO_CONSTANT, self.timeNanoSec - region.timeNanoSecs)
			)
			return
		if region.name in self.regionNames and self.timeNanoSec > region.timeNanoSecs:
			RosUtils.Logger().debug(
				"Region %s already exist and will not be replaced by older version. self.t = %d and region.t = %d" %
				(repr(region), self.timeNanoSec, region.timeNanoSecs)
			)
			return
		return super().addConnectedComponent(region)

	def intersection(self, other: "RegularDynamicRegion") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "RegularDynamicRegion") -> Set[str]:
		return super().union(other)

	def difference(self, other: "RegularDynamicRegion") -> Set[str]:
		return super().difference(other)

	def render(self, durationNs: int, envelopeColor: Union[RGBA, None] = None) -> Sequence[Marker]:
		return super().render(durationNs, envelopeColor)
