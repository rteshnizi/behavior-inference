import os
import pathlib
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	yamlPath = os.path.join(get_package_share_directory(packageName), "config", "case1.av1.yaml")

	nodes: List[Node] = [
		Node(
			package=packageName,
			namespace=packageName,
			executable="ESE",
			name="av1",
			parameters=[yamlPath],
		),
	]

	return LaunchDescription(nodes)
