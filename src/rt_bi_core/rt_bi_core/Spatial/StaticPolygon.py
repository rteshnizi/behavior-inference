from typing import Literal

from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Shared.Pose import Coords, CoordsList
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_commons.Utils.Msgs import Msgs
from rt_bi_core.Spatial.Polygon import Polygon


class StaticPolygon(Polygon):
	type: Literal[Polygon.Types.STATIC] = Polygon.Types.STATIC
	Type = Literal[Polygon.Types.STATIC]
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: CoordsList,
			predicates: list[Msgs.RtBi.Predicate] | Predicates,
			timeNanoSecs: int,
			envelopeColor: RGBA = ColorNames.WHITE,
			**kwArgs
		) -> None:
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			envelope=envelope,
			envelopeColor=envelopeColor,
			predicates=predicates,
			timeNanoSecs=timeNanoSecs,
			**kwArgs
		)
		return

	@property
	def centerOfRotation(self) -> Coords:
		return self.centroid
