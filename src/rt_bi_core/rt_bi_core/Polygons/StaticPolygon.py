from rt_bi_commons.Utils.RViz import ColorNames
from rt_bi_core.Polygons.Polygon import Polygon


class StaticPolygon(Polygon):
	type = Polygon.Types.STATIC
	def __init__(self, **kwArgs) -> None:
		super().__init__(
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.WHITE),
			**kwArgs
		)
		return
