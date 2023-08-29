from typing import Union

from rt_bi_core.Model.SensorRegion import SensorRegion
from rt_bi_core.Model.Tracklet import Tracklets
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose


class AffineSensorRegion(SensorRegion):
	def __init__(
			self,
			centerOfRotation: Pose,
			idNum: int,
			envelope: Geometry.CoordsList,
			fov: Union[Polygon, MultiPolygon, None]=None,
			tracks: Tracklets={},
			**kwArgs
		) -> None:
		"""
		Initialize an affine sensor region. An affine sensor region need's a center of rotation.

		Parameters
		----------
		centerOfRotation : Pose
			This will be used to interpolate the rotation motion of the region.
		idNum : int
			Id of the sensor region.
		envelope : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelope of the polygonal region.
		fov: Union[Polygon, MultiPolygon, None], default `None`
			The field-of-view, default forces construction using knowledge-base.
		tracks : Tracks, default `{}`
			The tracklets, as defined in the dissertation, observable in the fov,
			default forces construction using knowledge-base.
		"""
		super().__init__(
			idNum=idNum,
			envelope=envelope,
			tracks=tracks,
			fov=fov,
			**kwArgs
		)
		self.centerOfRotation = centerOfRotation

	@property
	def name(self) -> str:
		"""Attaches: `AFF-` to super `super().name`."""
		return "AFF-%s" % super().name
