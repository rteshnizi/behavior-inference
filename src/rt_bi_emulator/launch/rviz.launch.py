import os
import pathlib

from launch_ros.actions import Node

from launch import LaunchDescription

shareDir = str(pathlib.Path(__file__).parent.parent.resolve())

def generate_launch_description():
	return LaunchDescription([
		Node(
			package="rviz2",
			namespace="rviz2",
			executable="rviz2",
			name="rviz2_map",
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rviz.live.rviz")],
				"--ros-args",
				"--log-level",
				"warn",
			]
		),
	])
