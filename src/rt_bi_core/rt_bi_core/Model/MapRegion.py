from typing import Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import Color, KnownColors


class MapRegion(AffineRegion):
	def __init__(self, idNum: int, envelope: Geometry.CoordsList, **kwArgs) -> None:
		self.__featureDefinition: Union[Feature, None] = None
		super().__init__(
			centerOfRotation=Pose(0, 0, 0, 0),
			idNum=idNum,
			envelope=envelope,
			envelopeColor=kwArgs.pop("envelopeColor", KnownColors.GREY),
			regionType=AffineRegion.RegionType.MAP,
			timeNanoSecs=kwArgs.pop("timeNanoSecs", 0),
			interiorColor=kwArgs.pop("interiorColor", self.resolvedBgColor),
			**kwArgs
		)
		self.centerOfRotation = Geometry.toPose(self.interior.centroid, self.timeNanoSecs)

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

	def render(self, renderText = False) -> Sequence[Marker]:
		self.BACKGROUND_COLOR = self.resolvedBgColor
		return super().render(renderText, fill=True)
