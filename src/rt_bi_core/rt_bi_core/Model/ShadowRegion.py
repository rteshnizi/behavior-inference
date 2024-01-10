from typing import Literal, Sequence, Set, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import RGBA, ColorNames


class ShadowRegion(AffineRegion):
	def __init__(
		self,
		idNum: int,
		envelope: Geometry.CoordsList,
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
			centerOfRotation=kwArgs.pop("centerOfRotation", Pose(0, 0, 0, 0)),
			idNum=idNum,
			envelope=envelope,
			envelopeColor=ColorNames.BLACK,
			timeNanoSecs=kwArgs.pop("timeNanoSecs", 0),
			renderLineWidth=2,
			**kwArgs,
		)
		self.centerOfRotation = Geometry.toCoords(self.interior.centroid)
		self.__neighboringSensors: Set[str] = set()

	@property
	def regionType(self) -> Literal[AffineRegion.RegionType.SHADOW]:
		return self.RegionType.SHADOW

	def isNeighboringSensor(self, sensorRegionName: str) -> bool:
		return sensorRegionName in self.__neighboringSensors

	def addNeighboringSensors(self, sensorRegionName: str) -> None:
		self.__neighboringSensors.add(sensorRegionName)

	def render(self, renderText = False, envelopeColor: Union[RGBA, None] = None) -> Sequence[Marker]:
		return super().render(renderText=renderText, fill=True, envelopeColor=envelopeColor)
