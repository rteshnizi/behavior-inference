import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	yamlPath = os.path.join(get_package_share_directory(packageName), "config", "case1.avs.yaml")

	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="AV",
			name="av1",
			arguments= [
				"--ros-args",
				"--log-level",
				"info",
			],
			parameters=[yamlPath]
		),
		Node(
			package=packageName,
			namespace=packageName,
			executable="AV",
			name="av2",
			arguments= [
				"--ros-args",
				"--log-level",
				"info",
			],
			parameters=[yamlPath]
		),
	])