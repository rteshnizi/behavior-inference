from typing import Dict, List, Union

import rclpy

import rt_bi_utils.Ros as RosUtils
from rt_bi_core import MapServiceInterface
from rt_bi_core.Model.MapRegion import MapRegion
from rt_bi_core.Model.PolygonalRegion import PolygonalRegion
from rt_bi_core.Model.SensingRegion import SensingRegion
from rt_bi_core.ShadowTree.ShadowTree import ShadowTree
from rt_bi_utils import SaMsgs
from sa_msgs.msg import RobotState


class BehaviorAutomatonInterface(MapServiceInterface):
	"""
	This Node listens to all the messages published on the topics related to the Behavior Automaton.
	This node combines topic listeners and service clients.
	"""
	def __init__(self) -> None:
		""" Create a Behavior Automaton Interface node. """
		super().__init__(node_name="rt_bi_core_ba", subClass=True)
		self.get_logger().info("%s is starting..." % self.get_fully_qualified_name())
		RosUtils.SetLogger(self.get_logger())
		self.__shadowTree: ShadowTree = ShadowTree()
		self.__sensors: Union[Dict[int, SensingRegion], None] = None
		self.initializeMap()
		# start a timer to query the feature definitions every 5 secs
		self.create_timer(5, self.queryDefinitions)
		SaMsgs.subscribeToSaRobotStateTopic(self, self.__onSensingRegionUpdate)

	def render(self, regions: List[PolygonalRegion] = None) -> None:
		self.get_logger().info("Render BA...")
		return super().render(regions)

	def __updateMap(self, regions: List[MapRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Map update for BA %s" % repr(regions))
		self.__shadowTree.updateMap(timeNanoSecs=updateTime, regions=regions)
		return

	def __updateNamedRegions(self, regions: List[PolygonalRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Named region update for BA %s" % repr(regions))
		self.__shadowTree.updateNamedRegions(timeNanoSecs=updateTime, regions=regions)
		return

	def __updateSensingRegions(self, regions: List[SensingRegion] = []) -> None:
		updateTime = self.get_clock().now().nanoseconds
		self.get_logger().info("Sensing region update for BA %s" % repr(regions))
		self.__shadowTree.updateSensingRegions(timeNanoSecs=updateTime, regions=regions)
		return

	def __onSensingRegionUpdate(self, update: RobotState) -> None:
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
		self.__updateSensingRegions([sensor])
		return

def main(args=None):
	"""
	Start the Behavior Inference Run-time.
	"""
	rclpy.init(args=args)
	node = BehaviorAutomatonInterface()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
	return

if __name__ == "__main__":
	main()
