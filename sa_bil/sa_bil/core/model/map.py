from typing import Dict, List, Union
from shapely.geometry import Polygon
from sa_bil.core.model.featureMap import Feature
from sa_bil.core.model.mapRegion import MapRegion
from sa_bil.core.utils.geometry import Geometry, Polygon

class Map:
	def __init__(self, regionDefs: List[Dict[str, object]]):
		self.__regionDefs = regionDefs
		self.__regions: Union[Dict[str, MapRegion], None] = None
		self.__regions = self.regions
		self.__polygon: Union[Polygon, None] = None

	@property
	def regions(self) -> Dict[str, MapRegion]:
		if self.__regions is None:
			self.__regions = {}

			for regionDef in self.__regionDefs:
				feature = Feature(regionDef["terrain"], {
					"visibility_AV": regionDef["visibility_AV"],
					"traversability_car": regionDef["traversability_car"],
					"traversability_tank": regionDef["traversability_tank"]
				})
				self.__regions[regionDef["Name"]] = MapRegion(
					regionDef["Name"],
					regionDef["coordinates"],
					regionDef["terrain"],
					feature
					)
		return self.__regions

	@property
	def polygon(self) -> Polygon:
		if self.__polygon is None:
			polygons = [self.regions[r].polygon for r in self.regions]
			self.__polygon = Geometry.union(polygons)
		return self.__polygon

	def render(self, canvas):
		for region in self.regions.values():
			region.render(canvas)

	def clearRender(self, canvas):
		for region in self.regions.values():
			region.clearRender(canvas)
