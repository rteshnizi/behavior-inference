from typing import Union

from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.RViz import Color, KnownColors


class DynamicRegion(PolygonalRegion):
	"""
	Represents a dynamic polygonal region, that is, a time-dependant region.
	"""
	def __init__(
			self,
			idNum: int,
			envelope: Geometry.CoordsList,
			envelopeColor: Color,
			regionType: PolygonalRegion.RegionType,
			interior: Union[Polygon, MultiPolygon, None] = None,
			timeNanoSecs = 0,
			**kwArgs
		) -> None:
		"""
		Initialize a sensor region.

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
		timeNanoSecs: int, default `0`
			A timestamp representing absolute value of time in nanosecond (ns).
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
	def timeNanoSecs(self) -> float:
		"""If there is a timestamp associated, this returns the time, and `nan` otherwise."""
		return self.__timeNanoSecs
