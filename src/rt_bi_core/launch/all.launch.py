import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription

shareDir = str(pathlib.Path(__file__).parent.parent.resolve())

emulatorPackageName = "rt_bi_emulator"
corePackageName = "rt_bi_core"

def generate_launch_description():
	emulatorYamlPath = os.path.join(get_package_share_directory(emulatorPackageName), "config", "case1.avs.yaml")
	eventifierYamlPath = os.path.join(get_package_share_directory(corePackageName), "config", "ev.debug.yaml")
	EmulatorNodes = [
		Node(
			package=emulatorPackageName,
			namespace=emulatorPackageName,
			executable="MP",
			name="map",
		),
		Node(
			package=emulatorPackageName,
			namespace=emulatorPackageName,
			executable="AV",
			name="av1",
			parameters=[emulatorYamlPath],
		),
		Node(
			package=emulatorPackageName,
			namespace=emulatorPackageName,
			executable="AV",
			name="av2",
			parameters=[emulatorYamlPath],
		),
	]
	CoreNodes = [
		Node(
			package=corePackageName,
			namespace=corePackageName,
			executable="SI",
			name="si",
		),
		Node(
			package=corePackageName,
			namespace=corePackageName,
			executable="MI",
			name="mi",
		),
		Node(
			package=corePackageName,
			namespace=corePackageName,
			executable="EV",
			name="ev",
			arguments= [
				"--ros-args",
				"--log-level",
				"info",
			],
			parameters=[eventifierYamlPath],
		),
	]

	RVizNodes= [
		Node(
			package="rviz2",
			namespace="rviz2",
			executable="rviz2",
			name="rviz2_map",
			arguments=[
				"-d",
				[os.path.join(shareDir, "config", "rbc_map.rviz")],
			],
		),
	]

	return LaunchDescription(CoreNodes + EmulatorNodes + RVizNodes)
