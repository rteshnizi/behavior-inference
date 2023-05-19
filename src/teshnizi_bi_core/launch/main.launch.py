from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='teshnizi_bi_core',
			namespace='BI_RUNTIME',
			executable='main',
			name='main'
		),
		Node(
			package = 'sa_semantic_map',
			namespace='sa_map',
			executable='sa_map',
			name= 'map',
			output='both',
		)
	])
