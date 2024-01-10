from typing import Literal, Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.RViz import RGBA


class TargetRegion(AffineRegion):
	def __init__(
			self,
			centerOfRotation: Geometry.Coords,
			idNum: int,
			envelope: Geometry.CoordsList,
			envelopeColor: RGBA,
			timeNanoSecs: int,
			interior: Union[Polygon, MultiPolygon, None] = None,
			**kwArgs
		) -> None:
		super().__init__(
			centerOfRotation=centerOfRotation,
			idNum=idNum,
			envelope=envelope,
			envelopeColor=envelopeColor,
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)

	@property
	def regionType(self) -> Literal[AffineRegion.RegionType.TARGET]:
		return self.RegionType.TARGET

	def render(self, envelopeColor: Union[RGBA, None] = None) -> Sequence[Marker]:
		msgs = super().render(renderText=False, envelopeColor=envelopeColor)
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = super().clearRender()
		return msgs
