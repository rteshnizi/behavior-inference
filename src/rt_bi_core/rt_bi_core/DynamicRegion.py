from typing import Union

from rt_bi_commons.Utils.Geometry import Geometry, Polygon
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.PolygonalRegion import PolygonalRegion


class DynamicRegion(PolygonalRegion):
	"""
	Represents a dynamic polygonal region, that is, a time-dependant region.
	"""

	NANO_CONSTANT = 10 ** 9
	"""### Constant
	10 to the power of 9. Used in time conversion.
	"""

	DEFAULT_RENDER_DURATION = int(1.75 * NANO_CONSTANT)

	def __init__(
			self,
			id: str,
			envelope: Geometry.CoordsList,
			envelopeColor: RGBA,
			timeNanoSecs: int,
			interior: Union[Polygon, None] = None,
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
		interior: Union[Polygon, None], default `None`
			The interior of the region, if it is separate from its envelope, default forces construction.
		"""
		super().__init__(
			id=id,
			envelope=envelope,
			envelopeColor=envelopeColor,
			interior=interior,
			**kwArgs
		)
		self.__timeNanoSecs = timeNanoSecs

	@property
	def timeNanoSecs(self) -> int:
		"""The timestamp associated with the region in NanoSeconds."""
		return self.__timeNanoSecs

	@timeNanoSecs.setter
	def timeNanoSecs(self, t: int) -> None:
		self.__timeNanoSecs = t
		return
