from abc import ABC

from rt_bi_commons.Shared.Color import RGBA
from rt_bi_commons.Shared.Pose import Coords, CoordsList
from rt_bi_commons.Shared.Predicates import Predicates
from rt_bi_core.Spatial.Polygon import Polygon


class AffinePolygon(Polygon, ABC):
	"""The base class for all moving polygons."""
	def __init__(
			self,
			polygonId: str,
			regionId: str,
			envelope: CoordsList,
			envelopeColor: RGBA,
			centerOfRotation: Coords,
			timeNanoSecs: int,
			predicates: Predicates,
			**kwArgs
		) -> None:
		super().__init__(
			polygonId=polygonId,
			regionId=regionId,
			envelope=envelope,
			predicates=predicates,
			envelopeColor=envelopeColor,
			timeNanoSecs=timeNanoSecs,
			**kwArgs
		)
		self.__centerOfRotation = centerOfRotation

	@property
	def centerOfRotation(self) -> Coords:
		return self.__centerOfRotation
