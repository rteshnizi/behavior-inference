function rebuild-n-bil()
{
	cd ~/git/ros2_wrk
	colcon build --packages-up-to sa_bil
	source ./install/local_setup.sh
	ros2 launch sa_bil main.launch.py
}
