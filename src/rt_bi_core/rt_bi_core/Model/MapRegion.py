from typing import List, Union
from rt_bi_utils.RViz import Color, KnownColors
from visualization_msgs.msg import Marker

from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry
from sa_msgs.msg import PoseArray as SaPoseArray


class MapRegion(PolygonalRegion):
	def __init__(self, name: str, coords: SaPoseArray, typeStr: str):
		self.__featureDefinition: Union[Feature, None] = None
		coordList: Geometry.CoordsList = []
		for i in range(len(coords.traj)):
			coordList.append((coords.traj[i].x, coords.traj[i].y))
		super().__init__(name, coordList, KnownColors.GREY, self.resolvedBgColor)
		self.type = typeStr

	@property
	def resolvedBgColor(self) -> Color:
		return KnownColors.BLACK if self.isObstacle else KnownColors.TRANSPARENT

	@property
	def isObstacle(self) -> bool:
		return self.featureDefinition.traversability.car < 100

	@property
	def featureDefinition(self) -> Feature:
		if self.__featureDefinition is None:
			return Feature("Undefined")
		return self.__featureDefinition

	@featureDefinition.setter
	def featureDefinition(self, val: Feature) -> None:
		self.__featureDefinition = val
		_ = self.resolvedBgColor # This line is just here so that the background color is set accordingly.
		return

	def render(self, renderText = False) -> List[Marker]:
		self.BACKGROUND_COLOR = self.resolvedBgColor
		return super().render(renderText, fill=True)
