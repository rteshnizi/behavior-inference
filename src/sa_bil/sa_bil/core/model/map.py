from rclpy.impl.rcutils_logger import RcutilsLogger
from typing import Dict, List, Union
from shapely.geometry import Polygon
from sa_bil.core.model.featureMap import Feature
from sa_bil.core.model.mapRegion import MapRegion
from sa_bil.core.utils.geometry import Geometry, Polygon
from sa_msgs.msg import FeatureInfo

class Map:
	def __init__(self, logger: RcutilsLogger, regionDefs: FeatureInfo):
		self.logger: RcutilsLogger = logger
		self.__regionDefs: FeatureInfo = regionDefs
		self.__regions: Union[Dict[str, MapRegion], None] = None
		self.__regions = self.regions
		self.__polygon: Union[Polygon, None] = None

	@property
	def regions(self) -> Dict[str, MapRegion]:
		if self.__regions is not None: return self.__regions
		self.__regions = {}
		for i in range(len(self.__regionDefs.polygon_shape_list)):
			# FIXME: currently visibility_av has the content for both type and visibility_av
			# featureName = self.__regionDefs.type[i]
			featureName = self.__regionDefs.feature_name[i]
			visibilityAv = "self.__regionDefs.visibility_av[i]"
			traversabilityCar = self.__regionDefs.traversability_gv_car[i]
			traversabilityTank = self.__regionDefs.traversability_gv_tank[i]
			feature = Feature(featureName, {
					"visibility_av": visibilityAv,
					"traversability_gv_car": traversabilityCar,
					"traversability_gv_tank": traversabilityTank
			})
			regionName = self.__regionDefs.feature_name[i]
			coords = self.__regionDefs.polygon_shape_list[i]
			self.logger.info("Creating region %s..." % regionName)
			self.__regions["r%d" % i] = MapRegion(regionName, coords, featureName, feature)
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
