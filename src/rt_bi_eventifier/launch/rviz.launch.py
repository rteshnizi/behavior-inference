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
				[os.path.join(shareDir, "config", "rviz.cgr.rviz")],
				"--ros-args",
				"--log-level",
				"warn",
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
		# 		"--ros-args",
		# 		"--log-level",
		# 		"warn",
		# 	]
		# ),
		Node(
			package="rviz2",
			namespace="rviz2",
			executable="rviz2",
			name="rviz2_str", # cspell: disable-line
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rviz.igr.z.rviz")], # cspell: disable-line
				"--ros-args",
				"--log-level",
				"warn",
			]
		),
	])
