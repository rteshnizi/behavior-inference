import os
from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

shareDir = str(pathlib.Path(__file__).parent.parent.resolve())

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="rviz2",
			namespace="rviz2",
			executable="rviz2",
			name="main",
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rt_bi_core.rviz")],
			]
		),
	])
