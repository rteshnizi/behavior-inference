from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess

package_name = Path(__file__).parent.parent.name

def generate_launch_description():
	baYamlPath = str(Path(get_package_share_directory(package_name), "config", "ba.yaml"))

	return LaunchDescription([
		ExecuteProcess(
			cmd=["flask", "--app", "flaskApp", "run", "--host=0.0.0.0"],
			cwd=str(Path(get_package_share_directory(package_name), "launch")),
			name="FLASK",
		),
		Node(
			package="rosbridge_server",
			namespace="rosbridge_server",
			executable="rosbridge_websocket",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
		),
		Node(
			package=package_name,
			namespace=package_name,
			executable="BA",
			name="ba1",
			arguments= [
				"--ros-args",
				"--log-level",
				"warn",
			],
			parameters=[baYamlPath]
		),
	])
