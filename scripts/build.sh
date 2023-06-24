#!/bin/bash

while [ "$#" -gt 0 ]; do
	case $1 in
		-c|--clean)
			if [ -e ./install/local_setup.sh ]
			then
				cd "$(dirname "$0")" # change to the directory of the script
				cd .. # now we are in the workspace directory
				echo "Clean build..."
				echo "rm -rf build/ install/ log/"
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

cd "$(dirname "$0")" # change to the directory of the script
cd .. # now we are in the workspace directory

if [ -e ./install/local_setup.sh ]
then
	echo "colcon build --packages-up-to rt_bi_core"
	colcon build --packages-up-to rt_bi_core
	return $?
else
	echo "colcon build"
	colcon build
	return $?
fi

echo "Ready to launch..."
exit 0
