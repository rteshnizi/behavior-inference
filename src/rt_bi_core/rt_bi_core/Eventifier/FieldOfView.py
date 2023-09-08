from typing import List, Set, Union

from rt_bi_core.Model.AffineSensorRegion import AffineSensorRegion
from rt_bi_core.Model.RegularDynamicRegion import RegularDynamicRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import MultiPolygon, Polygon


class FieldOfView(RegularDynamicRegion):
	"""A Class to model Field-of-View."""
	def __init__(self, sensors: List[AffineSensorRegion] = []):
		"""
		A dictionary from `sensor.name` to the `AffineSensorRegion` object.
		"""
		super().__init__(regions=sensors)

	def __and__(self, other: "FieldOfView") -> Set[str]:
		return super().__and__(other)

	def __add__(self, other: "FieldOfView") -> Set[str]:
		return super().__add__(other)

	def __sub__(self, other: "FieldOfView") -> Set[str]:
		return super().__sub__(other)

	def __getitem__(self, regionName: str) -> AffineSensorRegion:
		return super().__getitem__(regionName)

	@property
	def fov(self) -> Union[Polygon, MultiPolygon]:
		return self.interior

	@property
	def tracks(self) -> Tracklets:
		tracks: Tracklets = {}
		for sensor in self.__regions.values():
			sensor: AffineSensorRegion = sensor
			tracks = { **tracks, **sensor.tracks }
		return tracks

	def addConnectedComponent(self, region: AffineSensorRegion) -> None:
		if region.regionType != AffineSensorRegion.RegionType.SENSING: return
		return super().addConnectedComponent(region)

	def intersection(self, other: "FieldOfView") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "FieldOfView") -> Set[str]:
		return super().union(other)

	def difference(self, other: "FieldOfView") -> Set[str]:
		return super().difference(other)
