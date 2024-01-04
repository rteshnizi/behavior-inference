from typing import Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import RGBA


class TargetRegion(AffineRegion):
	def __init__(
			self,
			centerOfRotation: Pose,
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
			regionType=AffineRegion.RegionType.TARGET,
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)

	def render(self, envelopeColor: Union[RGBA, None] = None) -> Sequence[Marker]:
		msgs = super().render(renderText=False, envelopeColor=envelopeColor)
		return msgs

	def clearRender(self) -> Sequence[Marker]:
		msgs = super().clearRender()
		return msgs
