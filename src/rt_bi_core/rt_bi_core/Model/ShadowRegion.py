from typing import List, Sequence, Set, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.DynamicRegion import DynamicRegion
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.RViz import Color, KnownColors


class ShadowRegion(DynamicRegion):
	def __init__(
		self,
		idNum: int,
		envelope: Geometry.CoordsList,
		timeNanoSecs: int,
		**kwArgs,
	):
		"""
		Initialize an polygonal region.

		Parameters
		----------
		idNum : int
			Id of the sensor region.
		envelope : Geometry.CoordsList
			The list of the coordinates of the vertices of the envelope of the polygonal region.
		timeNanoSecs: int
			A timestamp representing absolute value of time in nanosecond (ns).
		"""

		super().__init__(
			idNum=idNum,
			envelope=envelope,
			envelopeColor=KnownColors.BLACK,
			regionType=self.RegionType.SHADOW,
			timeNanoSecs=timeNanoSecs,
			renderLineWidth=2,
			**kwArgs,
		)
		self.__neighboringSensors: Set[str] = set()

	def isNeighboringSensor(self, sensorRegionName: str) -> bool:
		return sensorRegionName in self.__neighboringSensors

	def addNeighboringSensors(self, sensorRegionName: str) -> None:
		self.__neighboringSensors.add(sensorRegionName)

	def render(self, renderText = False, envelopeColor: Union[Color, None] = None) -> Sequence[Marker]:
		return super().render(renderText=renderText, fill=True, envelopeColor=envelopeColor)
