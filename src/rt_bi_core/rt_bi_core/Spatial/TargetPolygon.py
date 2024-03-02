from typing import Literal, Sequence

from visualization_msgs.msg import Marker

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Utils.RViz import RGBA
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon


class TargetPolygon(AffinePolygon):
	type: Literal[AffinePolygon.Types.TARGET] = AffinePolygon.Types.TARGET
	def __init__(self, **kwArgs) -> None:
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.ORANGE),
			**kwArgs
		)

	def render(self, envelopeColor: RGBA | None = None, durationNs: int = AffinePolygon.DEFAULT_RENDER_DURATION_NS) -> Sequence[Marker]:
		msgs = super().render(
			renderText=False,
			envelopeColor=envelopeColor,
			durationNs=durationNs
		)
		return msgs
