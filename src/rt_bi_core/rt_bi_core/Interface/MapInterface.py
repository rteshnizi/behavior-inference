from typing import Dict, Union

from rclpy.node import Node

from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_utils.Geometry import Geometry, Polygon
from rt_bi_utils.Ros import RosUtils
from sa_msgs.msg import FeatureInfo


class MapInterface(Node):
	"""The Viewer ROS Node"""
	def __init__(self):
		"""Create a Viewer ROS node."""
		super().__init__("rt_bi_core_mi")
		self.get_logger().info("Map Interface...")
		RosUtils.LOGGER = self.get_logger()
		self.__regions: Union[Dict[str, MapRegion], None] = None
		self.__regionDefs: Union[FeatureInfo, None] = None
		self.__polygon: Union[Polygon, None] = None
		self.__subscribeToTopics()

	@property
	def regions(self) -> Dict[str, MapRegion]:
		if self.__regions is not None: return self.__regions
		return {}

	@property
	def polygon(self) -> Polygon:
		if self.__polygon is None:
			polygons = [self.regions[r].polygon for r in self.regions]
			self.__polygon = Geometry.union(polygons)
		return self.__polygon

	def updateRegions(self, update: Union[FeatureInfo, None] = None) -> bool:
		if update is None: return False
		if (
			self.__regionDefs is not None and
			hash(repr(update)) == hash(repr(self.__regionDefs))
		):
			return False
		regions = {}
		self.__regionDefs = update
		self.get_logger().info("Updating region definitions...")
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
			self.get_logger().info("Creating region %s..." % regionName)
			regions["r%d" % i] = MapRegion(regionName, coords, featureName, feature)
		self.__regions = regions
		return True

	def __subscribeToTopics(self) -> None:
		# CreateSubscriber(self, FeatureInfo, "/sa_map/FeatureMap_BIL", self.mapUpdate)
		mapTopic= "/sa_map/FeatureMap_BIL"
		self.get_logger().info("Subscribe to %s" % mapTopic)
		self.create_subscription(FeatureInfo, mapTopic, self.mapUpdate, 10)
		return

	def __render(self):
		self.get_logger().info("Rendering Map...")
		for region in self.regions.values():
			region.render()

	def __clearRender(self):
		self.get_logger().info("CLEAR RENDER")
		return
		for region in self.regions.values():
			region.clearRender()

	def mapUpdate(self, msg: FeatureInfo) -> None:
		"""
		Callback function for the reception of map messages.
		"""
		updated = self.updateRegions(update=msg)
		if updated:
			self.__render()
		return
