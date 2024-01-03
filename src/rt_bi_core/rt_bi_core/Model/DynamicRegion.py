from typing import Union

from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.RViz import RGBA, RgbaNames


class DynamicRegion(PolygonalRegion):
	"""
	Represents a dynamic polygonal region, that is, a time-dependant region.
	"""

	NANO_CONSTANT = 10 ** 9
	"""### Constant
	10 to the power of 9. Used in time conversion.
	"""
	def __init__(
			self,
			idNum: int,
			envelope: Geometry.CoordsList,
			envelopeColor: RGBA,
			regionType: PolygonalRegion.RegionType,
			timeNanoSecs: int,
			interior: Union[Polygon, MultiPolygon, None] = None,
			**kwArgs
		) -> None:
		"""
		Initialize a sensor region.

		Parameters
		----------
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
			interior=interior,
			regionType=regionType,
			**kwArgs
		)
		self.__timeNanoSecs = timeNanoSecs

	@property
	def timeNanoSecs(self) -> int:
		"""The timestamp associated with the region in NanoSeconds."""
		return self.__timeNanoSecs
