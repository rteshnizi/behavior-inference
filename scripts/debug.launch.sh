#!/bin/bash

while [ "$#" -gt 0 ]; do
	case $1 in
		-c|--clean)
			if [ -e ./install/local_setup.sh ]
			then
				cd "$(dirname "$0")" # change to the directory of the script
				cd .. # now we are in the workspace directory
				echo "Clean build..."
				rm -rf build/ install/ log/
			fi
			shift ;;
		*)
			echo "Unknown arg: $1"
			echo "Known args:"
			echo "\t-c\t\t Clean build"
			exit 1 ;;
	esac
done

source_workspace_if_succeeded () {
	cd "$(dirname "$0")" # change to the directory of the script
	cd .. # now we are in the workspace directory
	echo "source $PWD/install/local_setup.sh"
	# https://stackoverflow.com/a/13702876/750567
	. $PWD/install/local_setup.sh
	if [ $? -eq 0 ]
	then
		echo "Sourced."
	else
		echo "Sourced Failed!"
		exit 1
	fi
}

cd "$(dirname "$0")" # change to the directory of the script
sh $PWD/build.sh

if [ $? -eq 0 ]
then
	source_workspace_if_succeeded
else
	echo "Build Failed!"
	exit 1
fi

cd "$(dirname "$0")" # change to the directory of the script
if [ $? -eq 0 ]
then
	concurrently --kill-others --names "Map,RViz,RBC" --prefix "[{name} - {time}]" -c "bgBlue.bold,bgGray.bold,bgYellow.bold" --hide "0" --timestamp-format "HH:mm:ss.SSS" "ros2 launch rt_bi_core sml.launch.py" "ros2 launch rt_bi_core rviz.launch.py" "ros2 launch rt_bi_core rbc.launch.py"
	exit 0
fi
exit 1
