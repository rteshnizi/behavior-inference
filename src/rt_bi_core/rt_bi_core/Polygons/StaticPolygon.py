from typing import Literal, Sequence

from visualization_msgs.msg import Marker

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

	def render(self, renderText = False) -> Sequence[Marker]:
		return super().render(renderText)
