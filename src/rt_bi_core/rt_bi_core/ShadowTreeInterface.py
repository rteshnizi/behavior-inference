from typing import Dict, Union

import rclpy
from bs4 import FeatureNotFound
from rclpy.node import Node

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_utils.Geometry import Geometry, Polygon
from sa_msgs.msg import FeatureInfo


class ShadowTreeInterface(Node):
	""" The Viewer ROS Node """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_core_shadow_tree")
		self.get_logger().info("Shadow Tree Interface is starting...")
		self.__MAP_UPDATE_TOPIC = "/sa_map/FeatureMap_BIL"
		self.__regions: Union[Dict[str, MapRegion], None] = None
		self.__regionDefs: Union[FeatureNotFound, None] = None
		self.__polygon: Union[Polygon, None] = None
		self.__createTopicPublishers()
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

	def __updateRegions(self, update: Union[FeatureInfo, None] = None) -> bool:
		# Edge cases
		if update is None:
			self.get_logger().warn("Received empty update!")
			return False
		if (self.__regionDefs is not None and hash(repr(update)) == hash(repr(self.__regionDefs))):
			return False

		# Normal cases
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

	def __createShadowTree(self, msg: FeatureInfo) -> None:
		"""
		Callback function for the reception of map messages.
		"""
		updated = self.__updateRegions(update=msg)
		if updated:
			self.__render()
		return

	def __subscribeToTopics(self) -> None:
		RosUtils.CreateSubscriber(self, FeatureInfo, self.__MAP_UPDATE_TOPIC, self.__createShadowTree)

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	shadowTreeNode = ShadowTreeInterface()
	rclpy.spin(shadowTreeNode)
	shadowTreeNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
