from typing import Dict, List, Sequence, Set, Union

from visualization_msgs.msg import Marker

from rt_bi_commons.Utils.Geometry import MultiPolygon, Polygon
from rt_bi_commons.Utils.RViz import ColorNames
from rt_bi_core.RegularAffineRegion import RegularAffineRegion
from rt_bi_core.SensorRegion import SensorRegion
from rt_bi_core.Tracklet import Tracklet


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
	def tracks(self) -> Dict[int, Tracklet]:
		tracks: Dict[int, Tracklet] = {}
		for n in self:
			sensor = self[n]
			for id in sensor.tracklets:
				if id not in tracks or tracks[id].timeNanoSecs < sensor.tracklets[id].timeNanoSecs:
					tracks[id] = sensor.tracklets[id]
		return tracks

	def addConnectedComponent(self, region: SensorRegion) -> None:
		if region.regionType != SensorRegion.RegionType.SENSING: raise TypeError(f"Incorrect region type {region.regionType}")
		return super().addConnectedComponent(region)

	def intersection(self, other: "FieldOfView") -> Set[str]:
		return super().intersection(other)

	def union(self, other: "FieldOfView") -> Set[str]:
		return super().union(other)

	def difference(self, other: "FieldOfView") -> Set[str]:
		return super().difference(other)

	def render(self) -> Sequence[Marker]:
		return super().render(durationNs=SensorRegion.DEFAULT_RENDER_DURATION, envelopeColor=ColorNames.GREEN)
