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
	--names "RUN, BA,MAP,AFF,SEN,TRG,ZEM,EVN,ZEV"\
	--prefix "[{name}-{time}]"\
	-c "bgWhite.black,bgBlue,bgMagenta,bgGreen,bgYellow.black,bgBlack.white,bgCyan.black,bgBlack.white"\
	--timestamp-format "HH:mm:ss"\
	"ros2 launch rt_bi_runtime all.launch.py"\
	"ros2 launch rt_bi_behavior ba.launch.py"\
	"ros2 launch rt_bi_emulator map.launch.py"\
	"ros2 launch rt_bi_emulator known.launch.py"\
	"ros2 launch rt_bi_emulator sensor.launch.py"\
	"ros2 launch rt_bi_emulator target.launch.py"\
	"ros2 launch rt_bi_emulator rviz.launch.py"\
	"ros2 launch rt_bi_eventifier ev.launch.py"\
	"ros2 launch rt_bi_eventifier rviz.launch.py"
exit 0
