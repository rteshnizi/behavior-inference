from typing import Generic, Sequence, TypeVar

from visualization_msgs.msg import Marker

from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.Polygons.DynamicPolygon import DynamicPolygon
from rt_bi_core.RegularRegions.SpatialRegion import SpatialRegion

RegionType = TypeVar("RegionType", bound=DynamicPolygon)

class DynamicRegion(Generic[RegionType], SpatialRegion[RegionType]):
	"""A Regular Spatial Region whose members are dynamic polygons."""

	MAX_UPDATE_DELAY_NS = 15 * DynamicPolygon.NANO_CONVERSION_CONSTANT
	"""
	### Core assumption:
	We allow at most a delay of some noticeable seconds between updates from a certain sensor before we declare it off.
	"""

	def __init__(self, regions: Sequence[RegionType] = []):
		super().__init__(regions=regions)

	def __and__(self, other: "DynamicRegion") -> set[DynamicPolygon.Id]:
		return super().__and__(other)

	def __add__(self, other: "DynamicRegion") -> set[DynamicPolygon.Id]:
		return super().__add__(other)

	def __sub__(self, other: "DynamicRegion") -> set[DynamicPolygon.Id]:
		return super().__sub__(other)

	def __getitem__(self, regionId: DynamicPolygon.Id) -> RegionType:
		return super().__getitem__(regionId)

	@property
	def timeNanoSec(self) -> int:
		if self.isEmpty: return -1
		else: return self[next(iter(self))].timeNanoSecs

	@timeNanoSec.setter
	def timeNanoSec(self, t: int) -> None:
		for rName in self.regionIds:
			self[rName].timeNanoSecs = t
		return

	def addConnectedComponent(self, region: RegionType) -> None:
		if not self.isEmpty and self.timeNanoSec - region.timeNanoSecs > self.MAX_UPDATE_DELAY_NS:
			Ros.Logger().debug(
				"Discarded old region %s. Given region is older than %fs. Time difference = %d ns." %
				(repr(region), self.MAX_UPDATE_DELAY_NS / DynamicPolygon.NANO_CONVERSION_CONSTANT, self.timeNanoSec - region.timeNanoSecs)
			)
			return
		if region.name in self.regionIds and self.timeNanoSec > region.timeNanoSecs:
			Ros.Logger().debug(
				"Region %s already exist and will not be replaced by older version. self.t = %d and region.t = %d" %
				(repr(region), self.timeNanoSec, region.timeNanoSecs)
			)
			return
		return super().addConnectedComponent(region)

	def intersection(self, other: "DynamicRegion") -> set[DynamicPolygon.Id]:
		return super().intersection(other)

	def union(self, other: "DynamicRegion") -> set[DynamicPolygon.Id]:
		return super().union(other)

	def difference(self, other: "DynamicRegion") -> set[DynamicPolygon.Id]:
		return super().difference(other)

	def render(self, durationNs: int = DynamicPolygon.DEFAULT_RENDER_DURATION_NS, envelopeColor: RGBA | None = None) -> Sequence[Marker]:
		return super().render(durationNs, envelopeColor)
