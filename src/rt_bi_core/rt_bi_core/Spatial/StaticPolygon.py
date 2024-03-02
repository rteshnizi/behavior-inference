from typing import Literal

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Shared.Pose import Coords, CoordsList
from rt_bi_commons.Utils.Geometry import Shapely
from rt_bi_commons.Utils.RViz import ColorNames
from rt_bi_core.Spatial.Polygon import Polygon


class StaticPolygon(Polygon):
	type: Literal[Polygon.Types.STATIC] = Polygon.Types.STATIC
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: CoordsList,
			envelopeColor: RGBA = ColorNames.WHITE,
			**kwArgs
		) -> None:
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			envelope=envelope,
			envelopeColor=envelopeColor,
			**kwArgs
		)
		return

	@property
	def centerOfRotation(self) -> Coords:
		return self.centroid
