import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

package_name = Path(__file__).parent.parent.name

def generate_launch_description():
	rdfYamlPath = os.path.join(get_package_share_directory(package_name), "config", "c3.dd.rdf.yaml")

	return LaunchDescription([
		# Node(
		# 	package=packageName,
		# 	namespace=packageName,
		# 	executable="BA",
		# 	name="ba1",
		# 	parameters=[baYamlPath]
		# ),
		Node(
			package=package_name,
			namespace=package_name,
			executable="DD_RDF",
			name="dd_rdf_1",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
			parameters=[rdfYamlPath],
		),
	])
