from typing import List, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.RViz import Color, KnownColors


class MapRegion(PolygonalRegion):
	def __init__(self, idNum: int, envelop: Geometry.CoordsList, **kwArgs) -> None:
		self.__featureDefinition: Union[Feature, None] = None
		super().__init__(
			idNum=idNum,
			envelop=envelop,
			envelopColor=KnownColors.GREY,
			interiorColor=self.resolvedBgColor,
			regionType=PolygonalRegion.RegionType.MAP,
			**kwArgs
		)

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
		return

	def render(self, renderText = False) -> List[Marker]:
		self.BACKGROUND_COLOR = self.resolvedBgColor
		return super().render(renderText, fill=True)
