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
			name="rviz2_cg",
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rviz.cg.rviz")],
			]
		),
		# Node(
		# 	package="rviz2",
		# 	namespace="rviz2",
		# 	executable="rviz2",
		# 	name="rviz2_ctcd", # cspell: disable-line
		# 	arguments=[
		# 		"-d",
		# 		[os.path.join(shareDir, "config", "rviz.ctcd.rviz")], # cspell: disable-line
		# 	]
		# ),
		Node(
			package="rviz2",
			namespace="rviz2",
			executable="rviz2",
			name="rviz2_str", # cspell: disable-line
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rviz.str.rviz")], # cspell: disable-line
			]
		),
	])
