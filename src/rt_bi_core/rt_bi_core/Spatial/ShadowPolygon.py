from typing import Literal

from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Shared.Pose import CoordsList
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils import Ros
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.StaticPolygon import StaticPolygon


class ShadowPolygon(StaticPolygon):
	type: Literal[StaticPolygon.Types.SHADOW] = StaticPolygon.Types.SHADOW
	Type = Literal[StaticPolygon.Types.SHADOW]
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: CoordsList,
			timeNanoSecs: int,
			predicates: Predicates,
			**kwArgs
		):
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			envelope=envelope,
			timeNanoSecs=timeNanoSecs,
			predicates=predicates,
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.ORANGE),
			renderLineWidth=kwArgs.pop("renderLineWidth", 2),
			**kwArgs,
		)

	def createMarkers(self, durationNs: int = -1, renderText: bool = False, envelopeColor: RGBA | None = None, stamped: bool = False) -> list[RViz.Msgs.Marker]:
		markers = super().createMarkers(durationNs, True, envelopeColor, stamped)
		if renderText:
			textCoords = self.centroid
			areaText = f"area={self.interior.area:.2f}"
			unstampedId = self.id if stamped else self.id.updateTime(0)
			timerMarker = RViz.createText(unstampedId, textCoords, areaText, ColorNames.RED, fontSize=10, durationNs=durationNs, idSuffix="area")
			Ros.AppendMessage(markers, timerMarker)
		return markers
