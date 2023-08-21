#!/bin/bash
fileName=$0

source_workspace_if_succeeded () {
	if [ "$PWD" != "$(dirname "$fileName")" ]; then cd "$(dirname "$fileName")"; fi # change to the directory of the script if necesseary
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

cd "$(dirname "$fileName")" # change to the directory of the script
. $PWD/helper.sh
parseArgs $@
parsingResult=$?

if [ $parsingResult -eq 1 ]
then # Unknown argument
	help
	exit 1
elif [ $(($parsingResult & 8)) -ne 8 ] # 8 means no build
then
	cd "$(dirname "$fileName")" # change to the directory of the script
	sh $PWD/build.sh $@			# pass-thru the arguments
fi

if [ $? -eq 0 ]
then
	source_workspace_if_succeeded
else
	echo "Build Failed!"
	exit 1
fi

cd "$(dirname "$fileName")" # change to the directory of the script
setEnvVars # set the necessary environment variables

cd .. # now we are in the workspace directory
concurrently\
	--kill-others\
	--names "RVZ,MAP,MI ,SI ,ST "\
	--prefix "[{name}-{time}]"\
	-c "bgWhite.black,bgBlueBright.black,bgBlue.black,bgYellow.black,bgMagentaBright.black"\
	--hide "0"\
	--timestamp-format "HH:mm:ss"\
	"ros2 launch rt_bi_core rviz.launch.py"\
	"ros2 launch rt_bi_emulator sml.launch.py"\
	"ros2 launch rt_bi_core mi.launch.py"\
	"ros2 launch rt_bi_core si.launch.py"\
	"ros2 launch rt_bi_core st.launch.py"
exit 0
