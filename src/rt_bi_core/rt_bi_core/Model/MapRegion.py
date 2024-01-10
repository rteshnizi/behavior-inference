from typing import Literal, Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.RViz import RGBA, ColorNames


class MapRegion(AffineRegion):
	def __init__(self, idNum: int, envelope: Geometry.CoordsList, timeNanoSecs: int = 0, interior: Union[Polygon, MultiPolygon, None] = None,**kwArgs) -> None:
		self.__featureDefinition: Union[Feature, None] = None
		super().__init__(
			centerOfRotation=(0.0, 0.0),
			idNum=idNum,
			envelope=envelope,
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.GREY),
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			interiorColor=kwArgs.pop("interiorColor", self.resolvedBgColor),
			**kwArgs
		)
		self.centerOfRotation = Geometry.toCoords(self.interior.centroid)

	@property
	def regionType(self) -> Literal[AffineRegion.RegionType.MAP]:
		return self.RegionType.MAP

	@property
	def resolvedBgColor(self) -> RGBA:
		return ColorNames.BLACK if self.isObstacle else ColorNames.TRANSPARENT

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

	def render(self, renderText = False) -> Sequence[Marker]:
		self.BACKGROUND_COLOR = self.resolvedBgColor
		return super().render(renderText, fill=True)
