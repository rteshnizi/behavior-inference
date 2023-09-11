from typing import Union

from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import Color


class AffineRegion(DynamicRegion):
	def __init__(
			self,
			centerOfRotation: Pose,
			idNum: int,
			envelope: Geometry.CoordsList,
			envelopeColor: Color,
			regionType: DynamicRegion.RegionType,
			timeNanoSecs: int,
			interior: Union[Polygon, MultiPolygon, None]=None,
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
		envelopeColor: Color
			The color of the envelope when/if rendered.
		regionType: RegionType, default `RegionType.BASE`
			The type of this region.
		timeNanoSecs: int
			A timestamp representing absolute value of time in nanosecond (ns).
		interior: Union[Polygon, MultiPolygon, None], default `None`
			The interior of the region, if it is separate from its envelope, default forces construction.
		"""
		super().__init__(
			idNum=idNum,
			envelope=envelope,
			envelopeColor=envelopeColor,
			regionType=regionType,
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)
		self.centerOfRotation = centerOfRotation

	@property
	def name(self) -> str:
		"""Attaches: `AFF-` to super `super().name`."""
		return "AFF-%s" % super().name
