from typing import Union

from rt_bi_commons.Utils.Geometry import Geometry, Polygon
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.DynamicRegion import DynamicRegion


class AffineRegion(DynamicRegion):
	def __init__(
			self,
			centerOfRotation: Geometry.Coords,
			idNum: int,
			envelope: Geometry.CoordsList,
			envelopeColor: RGBA,
			timeNanoSecs: int,
			interior: Union[Polygon, None]=None,
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
		interior: Union[Polygon, None], default `None`
			The interior of the region, if it is separate from its envelope, default forces construction.
		"""
		super().__init__(
			idNum=idNum,
			envelope=envelope,
			envelopeColor=envelopeColor,
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)
		self.centerOfRotation: Geometry.Coords = centerOfRotation

	@property
	def shortName(self) -> str:
		"""`AFF-super().name`."""
		return f"AFF-{super().shortName}"
