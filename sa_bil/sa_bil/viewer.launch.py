from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='sa_bil',
			namespace='sa_bil',
			executable='viewer',
			name='viewer'
		)
	])
