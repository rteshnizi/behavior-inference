import rclpy
from rclpy.node import Node
from sa_bil.core.parser import SemanticMapV2Parser
from sa_msgs.msg import EstimationMsg

class Viewer(Node):
	"""The Viewer ROS Node"""
	def __init__(self, mapPath: str = ""):
		"""
		Create a Viewer ROS node.

		Parameters
		----------
		mapPath : str, optional
			The path to the JSON file to be parsed, by default "".
		"""
		super().__init__("saBil_viewer")
		self.get_logger().info("Constructing Viewer...")
		self.__loadMap(mapPath)
		self.__subscribeToTopics()

	def __loadMap(self, mapPath: str) -> None:
		"""
		Load the map into the viewer.

		Parameters
		----------
		mapPath : str
			A string path to the map JSON file.

		Raises
		------
		RuntimeError
			THrows error if there is no file name.
		"""
		if mapPath == "": raise RuntimeError("Receiving map updates is not designed yet.")
		logger = self.get_logger()
		logger.info("Loading map...")
		envMap = SemanticMapV2Parser(mapPath, logger).parse()
		return

	def __subscribeToTopics(self) -> None:
		self.create_subscription(EstimationMsg, "topic", self.observationUpdate, 10)
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
		self.get_logger().info('I heard: "%s"' % msg)
		return

def main(args=None):
	"""
	Start the viewer.
	"""
	rclpy.init(args=args)
	viewer = Viewer("/home/reza/git/bil-ros2/sa_bil/sa_bil/mock/demo-march-2023.json")
	rclpy.spin(viewer)

if __name__ == "__main__":
	main()
