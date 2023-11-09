#!/bin/bash
. $(dirname "$0")/helper.sh
parseArgs $@
parsingResult=$?

cdScriptsDir
cd .. # now we are in the workspace directory
cleanBuild=$(( $parsingResult & 2 ))
symLink=$(( $parsingResult & 4 ))
noBuild=$(( $parsingResult & 8 ))

if [ $parsingResult -eq 1 ]
then # Unknown argument
	help
	exit 1
elif [ $noBuild -eq 8 ]
then
	echo "Exiting build script."
	exit 0
elif [ $cleanBuild -eq 2 ]
then
	echo "rm -rf build/ install/ log/"
	rm -rf build/ install/ log/
elif [ $symLink -eq 4 ]
then
	symLink=1
fi


if [ $symLink -eq 1 ]
then
	if [ -e ./install/local_setup.sh ]
	then
		echo "colcon build --packages-up-to rt_bi_core --symlink-install"
		colcon build --packages-up-to rt_bi_core --symlink-install
		exit $?
	else
		echo "colcon build --symlink-install"
		colcon build --symlink-install
		exit $?
	fi
else
	if [ -e ./install/local_setup.sh ]
	then
		echo "colcon build --packages-select-regex rt_bi_.*"
		colcon build --packages-select-regex rt_bi_.*
		exit $?
	else
		echo "colcon build"
		colcon build
		exit $?
	fi
fi

echo "Ready to launch..."
exit 0
