#!/bin/bash
. $(dirname "$0")/helper.sh
installRequirements
parseArgs $@
parsingResult=$?

if [ $parsingResult -eq 1 ]
then # Unknown argument
	help
	exit 1
elif [ $(($parsingResult & 8)) -ne 8 ] # 8 means no build
then
	cdScriptsDir
	/bin/bash $PWD/build.sh $@ # pass-thru the arguments
fi

if [ $? -eq 0 ]
then
	source_workspace
else
	echo "Build Failed!"
	exit 1
fi

setEnvVars # set the necessary environment variables

cdScriptsDir
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
