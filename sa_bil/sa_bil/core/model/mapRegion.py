from typing import List
from sa_bil.core.model.polygonalRegion import PolygonalRegion
from sa_bil.core.model.featureMap import Feature

class MapRegion(PolygonalRegion):
	def __init__(self, name: str, coords: List, typeStr: str, featureDefinition: Feature):
		self.isObstacle: bool = featureDefinition.traversabilty.car < 100
		super().__init__(name, coords, "Grey", "Black" if self.isObstacle else "")
		self.type = typeStr

	def render(self, canvas, renderText=False):
		super().render(canvas, renderText)
