from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sa_bil',
			namespace='sa_bil',
			executable='viewer',
			name='viewer'
		),
		Node(
			package = 'sa_semantic_map',
			namespace='sa_map',
			executable='sa_map',
			name= 'map',
			output='both',
		)
	])
