from typing import List, Union

from rt_bi_core.Model.AffineSensorRegion import AffineSensorRegion
from rt_bi_core.Model.RegularRegion import RegularSpatialRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import MultiPolygon, Polygon


class FieldOfView(RegularSpatialRegion):
	"""A Class to model Field-of-View."""
	def __init__(self, sensors: List[AffineSensorRegion] = []):
		"""
		A dictionary from `sensor.name` to the `AffineSensorRegion` object.
		"""
		super().__init__(regions=sensors)

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
