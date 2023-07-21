import os
from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib
from ament_index_python.packages import get_package_share_directory

packageName = pathlib.Path(__file__).parent.parent.name
shareDir = get_package_share_directory(packageName, print_warning=True)

def generate_launch_description():
	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="MapServiceInterface",
			name="msi"
		),
		Node(
			package=packageName,
			namespace=packageName,
			executable="MapTopicInterface",
			name="mti"
		),
		Node(
			package="rviz2",
			namespace="rviz2",
			executable="rviz2",
			name="rviz2",
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rt_bi_core.rviz")],
			]
		),
		Node(
			package = "sa_semantic_map",
			namespace="sa_map",
			executable="sa_map",
			name= "sa_map",
			output="both",
		),
	])
