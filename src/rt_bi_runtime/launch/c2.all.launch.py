import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	baYamlPath = os.path.join(get_package_share_directory(packageName), "config", "c2.ba.yaml")
	ddYamlPath = os.path.join(get_package_share_directory(packageName), "config", "c2.dd.yaml")

	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="BA",
			name="ba1",
			parameters=[baYamlPath]
		),
		Node(
			package=packageName,
			namespace=packageName,
			executable="DD",
			name="dd",
			parameters=[ddYamlPath]
		),
	])
