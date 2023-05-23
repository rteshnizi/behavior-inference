# from sa_bil.core.utils.geometry import Geometry # I put this on top here to assign shapely repr functions
import rclpy
from rclpy.node import Node
from sa_bil.core.model.map import Map
from sa_bil.core.parser import SemanticMapV2Parser
from sa_msgs.msg import EstimationMsg
from sa_msgs.msg import FeatureInfo
from typing import Union

class Viewer(Node):
	"""The Viewer ROS Node"""
	def __init__(self):
		"""
		Create a Viewer ROS node.

		Parameters
		----------
		mapPath : str, optional
			The path to the JSON file to be parsed, by default "".
		"""
		super().__init__("saBil_viewer")
		self.get_logger().info("Constructing Viewer...")
		self.parser: SemanticMapV2Parser = SemanticMapV2Parser(logger=self.get_logger())
		self.map: Union[Map, None] = None
		self.__subscribeToTopics()


	def __subscribeToTopics(self) -> None:
		self.create_subscription(FeatureInfo, "/sa_map/FeatureMap_Viz", self.viewerUpdate, 10)
		self.create_subscription(FeatureInfo, "/sa_map/FeatureMap_BIL", self.mapUpdate, 10)
		return

	def mapUpdate(self, msg: FeatureInfo) -> None:
		"""
		Callback function for the reception of map messages.
		"""
		if (self.map is None):
			self.get_logger().info("Initializing map...")
			self.map = self.parser.parse(msg)
		self.map.render()
		return

	def viewerUpdate(self, msg: FeatureInfo) -> None:
		"""
		Callback function for the reception of messages to update the viewer.
		sa_msgs/PoseArray[] polygon_shape_list
		```
		string[] feature_name
		int64[] traversability_gv_car
		int64[] traversability_gv_tank

		string[] type
		string[] visibility_av
		```
		"""
		self.get_logger().info("Vwr Msg")
		return

	def observationUpdate(self, msg: EstimationMsg) -> None:
		"""
		Callback function for the reception of trajectory estimation messages.

		Parameters
		----------
		msg : EstimationMsg
		```python
		float detection_time
		sa_msgs/RobotState robot_state
		sa_msgs/PoseEstimation[] pose_estimations
		int discrete_detections
		```
		"""
		self.get_logger().info("EM")
		return

def main(args=None):
	"""
	Start the viewer.
	"""
	rclpy.init(args=args)
	viewer = Viewer()
	rclpy.spin(viewer)

if __name__ == "__main__":
	main()
