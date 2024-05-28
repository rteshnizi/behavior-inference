from typing import Literal

from rt_bi_commons.Shared.Color import RGBA, ColorNames
from rt_bi_commons.Shared.Pose import Coords, CoordsList
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_core.Spatial.Polygon import Polygon


class DynamicPolygon(Polygon):
	type: Literal[Polygon.Types.DYNAMIC] = Polygon.Types.DYNAMIC
	Type = Literal[Polygon.Types.DYNAMIC]
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
			envelopeColor=kwArgs.pop("envelopeColor", ColorNames.YELLOW),
			predicates=predicates,
			timeNanoSecs=timeNanoSecs,
			hIndex=hIndex,
			**kwArgs
		)
		return

	@property
	def centerOfRotation(self) -> Coords:
		return self.centroid

	@property
	def envelopeColor(self) -> RGBA:
		"""The color used to render the boundary of this polygon."""
		if self.isAccessible: return super().envelopeColor
		return ColorNames.GREY
