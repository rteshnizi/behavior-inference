from typing import Dict, List, Union

import rclpy
from rclpy.node import Node
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensingRegion import SensingRegion
from rt_bi_utils.RViz import RViz
from visualization_msgs.msg import MarkerArray

from rt_bi_utils.Geometry import Geometry, Polygon
from rt_bi_utils.SaMsgs import SaMsgs
from sa_msgs.msg import FeatureInfo, RobotState


class SensorTopicInterface(Node):
	""" This Node listens to all the messages published on the topics related to sensors and renders them. """
	def __init__(self):
		""" Create a Viewer ROS node. """
		super().__init__("rt_bi_core_map")
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		self.__regions: Union[Dict[str, PolygonalRegion], None] = None
		self.__sensors: Union[Dict[int, SensingRegion], None] = None
		self.__regionDefs: Union[FeatureInfo, None] = None
		self.__polygon: Union[Polygon, None] = None
		SaMsgs.subscribeToSaRobotStateTopic(self, self.__fovUpdate)
		(self.__rvizPublisher, _) = RViz.createRVizPublisher(self)

	@property
	def regions(self) -> Dict[str, PolygonalRegion]:
		if self.__regions is not None: return self.__regions
		return {}

	@property
	def polygon(self) -> Polygon:
		if self.__polygon is None:
			polygons = [self.regions[r].polygon for r in self.regions]
			self.__polygon = Geometry.union(polygons)
		return self.__polygon

	def __clearRender(self):
		self.get_logger().warn("%s is not implemented." % self.__clearRender.__name__)
		return
		for region in self.regions.values():
			region.clearRender()

	def __fovUpdate(self, update: RobotState) -> None:
		if update is None:
			self.get_logger().warn("Received empty RobotState!")
			return False
		if (
			self.__sensors is not None and
			update.robot_id in self.__sensors and
			hash(repr(update)) == hash(repr(self.__sensors[update.robot_id]))
		):
			return False

		self.get_logger().info("Updating sensor %d definition..." % update.robot_id)
		if self.__sensors is None:
			self.__sensors = {}
		coords = SaMsgs.convertSaPoseListToCoordsList(update.fov.corners)
		sensor = SensingRegion("S-%d" % update.robot_id, coords, self.get_clock().now().nanoseconds, update.robot_id)
		self.__sensors[update.robot_id] = sensor
		self.__render([sensor])
		return

	def __render(self, regions: List[PolygonalRegion] = None):
		if not RViz.isRVizReady(self, self.__rvizPublisher):
			self.get_logger().warn("Skipping map render... RViz is not ready yet to receive messages.")
			return
		if regions is None:
			self.get_logger().info("Rendering map...")
			regionList = self.regions.values()
		else:
			self.get_logger().info("Rendering regions %s..." % repr([r.name for r in regions]))
			regionList = regions
		message = MarkerArray()
		for region in regionList:
			message.markers += region.render()
		self.get_logger().info("MarkerArray about to be sent with %d markers." % len(message.markers))
		self.__rvizPublisher.publish(message)
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = SensorTopicInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
