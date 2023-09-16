from typing import List, Set

from visualization_msgs.msg import Marker

from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.RViz import KnownColors


class ShadowRegion(PolygonalRegion):
	def __init__(
		self,
		idNum: int,
		envelope: Geometry.CoordsList,
		**kwArgs
	):
		"""
		Initialize an polygonal region.

		Parameters
		----------
		idNum : int
			Id of the sensor region.
		envelope : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelope of the polygonal region.
		"""

		super().__init__(
			idNum=idNum,
			envelope=envelope,
			envelopeColor=KnownColors.BLACK,
			regionType=PolygonalRegion.RegionType.SHADOW,
			**kwArgs
		)
		self.__neighboringSensors: Set[str] = set()

	def isNeighboringSensor(self, sensorRegionName: str) -> bool:
		return sensorRegionName in self.__neighboringSensors

	def addNeighboringSensors(self, sensorRegionName: str) -> None:
		self.__neighboringSensors.add(sensorRegionName)

	def render(self, renderText=True) -> List[Marker]:
		return super().render(renderText=renderText, fill=True)
