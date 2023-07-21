from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_utils.Geometry import Geometry
from rt_bi_utils.RViz import KnownColors

class ValidatorRegion(PolygonalRegion):
	"""
	coords will be used to create the polygon.
	If polygon is given, coords arg will be ignored.
	"""
	def __init__(self, name: str, coords: Geometry.CoordsList, inFov=False):
		super().__init__(name, coords, KnownColors.RED if inFov else KnownColors.BLUE)
		self.inFov = inFov

	def render(self, canvas):
		super().render(canvas, renderText=True)
