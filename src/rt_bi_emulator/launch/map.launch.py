import pathlib

from launch_ros.actions import Node

from launch import LaunchDescription

packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="RMP",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
		),
		Node(
			package=packageName,
			namespace=packageName,
			executable="EMP",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
		),
	])
