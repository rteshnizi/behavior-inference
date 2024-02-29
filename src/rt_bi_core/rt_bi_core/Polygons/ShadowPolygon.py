from typing import Literal, Sequence

from visualization_msgs.msg import Marker

from rt_bi_commons.Utils.Geometry import GeometryLib
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_commons.Utils.RViz import RGBA, ColorNames
from rt_bi_core.Polygons.DynamicPolygon import DynamicPolygon


class ShadowPolygon(DynamicPolygon):
	def __init__(self, **kwArgs):
		super().__init__(
			timeNanoSecs=kwArgs.pop("timeNanoSecs", 0.0),
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.BLACK),
			renderLineWidth=kwArgs.pop("renderLineWidth", 2),
			**kwArgs,
		)

	@property
	def regionType(self) -> Literal[DynamicPolygon.Types.SHADOW]:
		return self.Types.SHADOW

	def render(self, renderText = False, envelopeColor: RGBA | None = None, durationNs: int = DynamicPolygon.DEFAULT_RENDER_DURATION_NS) -> Sequence[Marker]:
		return super().render(renderText=renderText, envelopeColor=envelopeColor, durationNs=durationNs)
