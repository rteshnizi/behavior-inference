from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="GvEmulator",
			name="main"
		),
	])
