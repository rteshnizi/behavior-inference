from typing import Literal

from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.AffinePolygonBase import AffinePolygonBase


class AffinePolygon(AffinePolygonBase):
	"""A mobile polygon with a timestamp."""

	type: Literal[AffinePolygonBase.Types.AFFINE] = AffinePolygonBase.Types.AFFINE
	Type = Literal[AffinePolygonBase.Types.AFFINE]
	def __init__(self, **kwArgs) -> None:
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.PURPLE),
			**kwArgs
		)

	def createMarkers(self, durationNs: int = AffinePolygonBase.DEFAULT_RENDER_DURATION_NS, renderText: bool = False, envelopeColor: RGBA | None = None, stamped: bool = True) -> list[RViz.Msgs.Marker]:
		return super().createMarkers(durationNs, renderText, envelopeColor, stamped)
