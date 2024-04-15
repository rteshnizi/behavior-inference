import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

package_name = Path(__file__).parent.parent.name

def generate_launch_description():
	baYamlPath = os.path.join(get_package_share_directory(package_name), "config", "ba.yaml")

	return LaunchDescription([
		Node(
			package=package_name,
			namespace=package_name,
			executable="BA",
			name="ba1",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
			parameters=[baYamlPath]
		),
		Node(
			package=package_name,
			namespace=package_name,
			executable="BA_RENDERER",
			arguments= [
				"--ros-args",
				"--log-level",
				"info",
			],
		),
	])
