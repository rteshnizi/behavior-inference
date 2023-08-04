from typing import List

import rclpy
from rt_bi_core import MapServiceInterface, SensorTopicInterface
from rt_bi_utils import SaMsgs

import rt_bi_utils.Ros as RosUtils
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensingRegion import SensingRegion
from rt_bi_core.ShadowTree.ShadowTree import ShadowTree


class ShadowTreeInterface(MapServiceInterface, SensorTopicInterface):
	"""
	This Node listens to all the messages published on the topics related to the shadow tree.
	This node combines topic listeners and service clients.
	"""
	def __init__(self) -> None:
		""" Create a Shadow Tree Interface node. """
		super().__init__(node_name="rt_bi_core_shadow_tree", subClass=True)
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__shadowTree: ShadowTree = ShadowTree()
		self.initializeMap()
		# start a timer to query the feature definitions every 5 secs
		self.create_timer(5, self.queryDefinitions)
		self.__mapClient = SaMsgs.createSaFeatureQueryClient(self)
		SaMsgs.subscribeToSaRobotStateTopic(self, self.__onFovUpdate)

	def render(self, regions: List[PolygonalRegion] = None) -> None:
		return super().render(regions)

	def __updateMap(self, regions: List[MapRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Map update for shadow tree %s" % repr(regions))
		self.__shadowTree.updateMap(timeNanoSecs=updateTime, regions=regions)
		return

	def __updateNamedRegions(self, regions: List[PolygonalRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Named update for shadow tree %s" % repr(regions))
		self.__shadowTree.updateNamedRegions(timeNanoSecs=updateTime, regions=regions)
		return

	def __updateSensingRegions(self, regions: List[SensingRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("FOV update for shadow tree %s" % repr(regions))
		self.__shadowTree.updateSensingRegions(timeNanoSecs=updateTime, regions=regions)
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = ShadowTreeInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
