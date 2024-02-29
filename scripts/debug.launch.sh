#!/bin/bash
. $(dirname "$0")/helper.sh
setEnvVars # set the necessary environment variables
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
	sourceWorkspace
else
	echo "${Red}Build Failed!${Color_Off}"
	exit 1
fi

cdScriptsDir
cd .. # now we are in the workspace directory
echo
echo

concurrently\
	--kill-others-on-fail\
	--names "MAP,DYN,SEN,TRG,EMZ,RUN"\
	--prefix "[{name}-{time}]"\
	-c "bgWhite.black,bgGreen,bgBlue,bgRed.inverse,bgYellow.black,bgMagenta,bgYellow.black,bgCyan.black"\
	--timestamp-format "HH:mm:ss"\
	"ros2 launch rt_bi_emulator map.launch.py"\
	"ros2 launch rt_bi_emulator dynamic.launch.py"\
	"ros2 launch rt_bi_emulator sensor.launch.py"\
	"ros2 launch rt_bi_emulator target.launch.py"\
	"ros2 launch rt_bi_emulator rviz.launch.py"\
	"ros2 launch rt_bi_runtime c3.all.launch.py"
	# --names "MAP,DYN,SEN,TRG,EMZ,EV ,EVZ,RUN"\
	# "ros2 launch rt_bi_eventifier ev.launch.py"\
	# "ros2 launch rt_bi_eventifier rviz.launch.py"\
exit 0
