from typing import List

from visualization_msgs.msg import Marker

from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.RViz import KnownColors


class SymbolRegion(PolygonalRegion):
	def __init__(self, idNum: int, envelope: Geometry.CoordsList, inFov=False, **kwArgs):
		super().__init__(
			idNum=idNum,
			envelope=envelope,
			envelopeColor=KnownColors.RED if inFov else KnownColors.BLUE,
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

	def render(self) -> List[Marker]:
		return super().render(renderText=True)
