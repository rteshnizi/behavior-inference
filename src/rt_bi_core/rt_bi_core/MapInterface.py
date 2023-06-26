from typing import Dict, Union

import rclpy
from rclpy.node import Node, Publisher
from visualization_msgs.msg import MarkerArray

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.FeatureMap import Feature
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_utils.Geometry import Geometry, Polygon
from sa_msgs.msg import FeatureInfo


class MapInterface(Node):
	""" The Viewer ROS Node """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_core_map")
		self.get_logger().info("Map Interface is starting...")
		self.__rvizPublisher: Publisher
		self.__MAP_UPDATE_TOPIC = "/sa_map/FeatureMap_BIL"
		self.__RVIZ_TOPIC = RosUtils.CreateTopicName("rbc_markers")
		""" https://wiki.ros.org/rviz/DisplayTypes/Marker#line-8 """
		self.__regions: Union[Dict[str, MapRegion], None] = None
		self.__regionDefs: Union[FeatureInfo, None] = None
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
		if not self.__isRVizReady():
			self.get_logger().warn("Skipping map update... RViz is not ready yet.")
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

	def __subscribeToTopics(self) -> None:
		RosUtils.CreateSubscriber(self, FeatureInfo, self.__MAP_UPDATE_TOPIC, self.__mapUpdate)

	def __createTopicPublishers(self) -> None:
		(self.__rvizPublisher, _) = RosUtils.CreatePublisher(self, MarkerArray, self.__RVIZ_TOPIC)

	def __isRVizReady(self) -> bool:
		if any(n for n in self.executor.get_nodes() if n.get_name().lower().find("rviz") > -1):
			self.get_logger().warn("No node containing the name RViz was found.")
			return False
		if self.__rvizPublisher.get_subscription_count() == 0:
			self.get_logger().warn("No subscribers to visualization messages.")
			return False
		return True

	def __render(self):
		self.get_logger().info("Rendering map...")
		message = MarkerArray()
		for region in self.regions.values():
			message.markers += region.render()
		RosUtils.Logger().info("MarkerArray about to be sent with %d markers." % len(message.markers))
		self.__rvizPublisher.publish(message)
		return

	def __mapUpdate(self, msg: FeatureInfo) -> None:
		"""
		Callback function for the reception of map messages.
		"""
		updated = self.__updateRegions(update=msg)
		if updated:
			self.__render()
		return

	def __clearRender(self):
		self.get_logger().warn("__clearRender is not implemented.")
		return
		for region in self.regions.values():
			region.clearRender()

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	mapNode = MapInterface()
	rclpy.spin(mapNode)
	mapNode.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
