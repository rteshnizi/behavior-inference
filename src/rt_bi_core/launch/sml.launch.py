from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

shareDir = str(pathlib.Path(__file__).parent.parent.resolve())

def generate_launch_description():
	return LaunchDescription([
		Node(
			package = "sa_semantic_map",
			namespace="sa_map",
			executable="sa_map",
			name= "sa_map",
			output="both",
		),
	])
