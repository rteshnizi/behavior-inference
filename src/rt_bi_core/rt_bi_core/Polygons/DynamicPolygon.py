from abc import ABC

from rt_bi_commons.Shared.Color import ColorNames
from rt_bi_core.Polygons.Polygon import Polygon


class DynamicPolygon(Polygon, ABC):
	"""A Polygon with a timestamp."""

	type = Polygon.Types.DYNAMIC
	def __init__(self, **kwArgs) -> None:
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.WHITE),
			**kwArgs
		)
