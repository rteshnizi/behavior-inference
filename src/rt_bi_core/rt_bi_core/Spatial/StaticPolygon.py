from typing import Literal

from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Shared.Pose import Coords, CoordsList
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils.RViz import RViz
from rt_bi_core.Spatial.Polygon import Polygon


class StaticPolygon(Polygon):
	type: Literal[Polygon.Types.STATIC] = Polygon.Types.STATIC
	Type = Literal[Polygon.Types.STATIC]
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			subPartId: str,
			envelope: CoordsList,
			predicates: Predicates,
			timeNanoSecs: int,
			hIndex: int,
			**kwArgs
		) -> None:
		kwArgs.pop("centerOfRotation", None)
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			subPartId=subPartId,
			envelope=envelope,
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.WHITE),
			predicates=predicates,
			timeNanoSecs=timeNanoSecs,
			hIndex=hIndex,
			**kwArgs
		)
		return

	@property
	def centerOfRotation(self) -> Coords:
		return self.centroid

	def createMarkers(self, durationNs: int = Polygon.DEFAULT_RENDER_DURATION_NS, renderText: bool = False, envelopeColor: RGBA | None = None, stamped: bool = False) -> list[RViz.Msgs.Marker]:
		return super().createMarkers(durationNs, renderText, envelopeColor, stamped)
