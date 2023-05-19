from typing import List
from shapely.geometry import Polygon
from sa_bil.core.model.polygonalRegion import PolygonalRegion

class ShadowRegion(PolygonalRegion):
	def __init__(self, name, coords, polygon: Polygon = None):
		super().__init__(name, coords, boundaryColor="BLACK", backgroundColor="GRAY", polygon=polygon)
		self._neighboringSensors: List[str] = []

	@property
	def neighboringSensors(self):
		return self._neighboringSensors

	def addNeighboringSensors(self, sensorName):
		self._neighboringSensors.append(sensorName)

	def render(self, canvas, renderText=True):
		super().render(canvas, renderText, hashFill=True, hashDensity=50)
