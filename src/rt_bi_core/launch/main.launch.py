import os
from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

#                          py file < launch < pack < share
shareDir = str(pathlib.Path(__file__).parent.parent.parent.resolve())
packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="MapInterface",
			name="main"
		),
		Node(
			package="rviz2",
			namespace="rt_bi_core",
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
