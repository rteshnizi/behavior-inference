from typing import Literal, Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_commons.Utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.AffineRegion import AffineRegion


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

	def render(self, envelopeColor: Union[RGBA, None] = None, durationNs: int = -1) -> Sequence[Marker]:
		msgs = super().render(renderText=False, envelopeColor=envelopeColor, durationNs=durationNs)
		return msgs
