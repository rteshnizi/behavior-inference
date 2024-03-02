from typing import Literal, Sequence

from visualization_msgs.msg import Marker

from rt_bi_commons.Shared.Pose import CoordsList
from rt_bi_commons.Utils.RViz import RGBA, ColorNames
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon


class ShadowPolygon(StaticPolygon):
	type: Literal[StaticPolygon.Types.SHADOW] = StaticPolygon.Types.SHADOW
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: CoordsList,
			**kwArgs
		):
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			envelope=envelope,
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.BLACK),
			renderLineWidth=kwArgs.pop("renderLineWidth", 2),
			**kwArgs,
		)

	def render(self, renderText = False, envelopeColor: RGBA | None = None, durationNs: int = StaticPolygon.DEFAULT_RENDER_DURATION_NS) -> Sequence[Marker]:
		return super().render(renderText=renderText, envelopeColor=envelopeColor, durationNs=durationNs)
