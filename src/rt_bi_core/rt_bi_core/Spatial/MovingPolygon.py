from typing import Literal

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_core.Spatial.AffinePolygon import AffinePolygon


class MovingPolygon(AffinePolygon):
	"""A Polygon with a timestamp."""

	type: Literal[AffinePolygon.Types.MOVING] = AffinePolygon.Types.MOVING
	Type = Literal[AffinePolygon.Types.MOVING]
	def __init__(self, **kwArgs) -> None:
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.PURPLE),
			**kwArgs
		)
