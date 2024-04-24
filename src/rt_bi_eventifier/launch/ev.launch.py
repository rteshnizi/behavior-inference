import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

corePackageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	eventifierYamlPath = os.path.join(get_package_share_directory(corePackageName), "config", "ev.yaml")
	return LaunchDescription([
		Node(
			package=corePackageName,
			namespace=corePackageName,
			executable="EV",
			name="eventifier",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
			parameters=[eventifierYamlPath],
		),
	])
