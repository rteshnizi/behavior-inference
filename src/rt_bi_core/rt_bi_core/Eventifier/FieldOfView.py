from typing import List, Set, Union

from rt_bi_core.Model.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import MultiPolygon, Polygon


class FieldOfView(RegularAffineRegion[SensorRegion]):
	"""A Class to model Field-of-View."""
	def __init__(self, sensors: List[SensorRegion] = []):
		"""
		A dictionary from `sensor.name` to the `SensorRegion` object.
		"""
		super().__init__(regions=sensors)

	def __and__(self, other: "FieldOfView") -> Set[str]:
		return super().__and__(other)

	def __add__(self, other: "FieldOfView") -> Set[str]:
		return super().__add__(other)

	def __sub__(self, other: "FieldOfView") -> Set[str]:
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> SensorRegion:
		return super().__getitem__(regionName)

	@property
	def fov(self) -> Union[Polygon, MultiPolygon]:
		return self.interior

	@property
	def tracks(self) -> Tracklets:
		tracks: Tracklets = {}
		for n in self:
			sensor = self[n]
			tracks = { **tracks, **sensor.tracks }
		return tracks

	def addConnectedComponent(self, region: SensorRegion) -> None:
		if region.regionType != SensorRegion.RegionType.SENSING: return
		return super().addConnectedComponent(region)

	def intersection(self, other: "FieldOfView") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "FieldOfView") -> Set[str]:
		return super().union(other)

	def difference(self, other: "FieldOfView") -> Set[str]:
		return super().difference(other)
