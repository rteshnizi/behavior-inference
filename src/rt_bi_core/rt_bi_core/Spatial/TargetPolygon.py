from typing import Literal

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon


class TargetPolygon(AffinePolygon):
	type: Literal[AffinePolygon.Types.TARGET] = AffinePolygon.Types.TARGET
	Type = Literal[AffinePolygon.Types.TARGET]
	def __init__(self, **kwArgs) -> None:
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.ORANGE),
			**kwArgs
		)

	def createMarkers(self, durationNs: int = AffinePolygon.DEFAULT_RENDER_DURATION_NS, renderText: bool = False, stamped: bool = True) -> list[RViz.Msgs.Marker]:
		msgs = super().createMarkers(durationNs=durationNs, renderText=renderText, stamped=stamped)
		return msgs
