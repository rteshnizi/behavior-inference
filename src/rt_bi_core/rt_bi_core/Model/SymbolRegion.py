from typing import Sequence, Union

from visualization_msgs.msg import Marker

from rt_bi_core.Model.AffineRegion import AffineRegion
from rt_bi_utils.Geometry import Geometry, MultiPolygon, Polygon
from rt_bi_utils.Pose import Pose
from rt_bi_utils.RViz import RgbaNames


class SymbolRegion(AffineRegion):
	def __init__(self, centerOfRotation: Pose, idNum: int, envelope: Geometry.CoordsList, timeNanoSecs: int, interior: Union[Polygon, MultiPolygon, None]=None, inFov=False, **kwArgs):
		super().__init__(
			centerOfRotation=centerOfRotation,
			idNum=idNum,
			envelope=envelope,
			envelopeColor=RgbaNames.RED if inFov else RgbaNames.BLUE,
			regionType=AffineRegion.RegionType.SYMBOL,
			timeNanoSecs=timeNanoSecs,
			interior=interior,
			**kwArgs
		)
		self.__inFov = inFov

	@property
	def inFov(self) -> bool:
		"""Indicates whether this region is inside the field of view."""
		return self.__inFov

	@inFov.setter
	def inFov(self, _: bool) -> None:
		raise NotImplementedError("No setter.")

	def render(self) -> Sequence[Marker]:
		return super().render(renderText=True)
