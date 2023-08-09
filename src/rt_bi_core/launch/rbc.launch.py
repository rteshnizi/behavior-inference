import pathlib

from launch_ros.actions import Node

from launch import LaunchDescription

packageName = pathlib.Path(__file__).parent.parent.name

def generate_launch_description():
	return LaunchDescription([
		Node(
			package=packageName,
			namespace=packageName,
			executable="MI",
			name="MI"
		),
		Node(
			package=packageName,
			namespace=packageName,
			executable="SI",
			name="SI"
		),
	])
