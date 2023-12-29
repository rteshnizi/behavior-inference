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
	echo "${Red}Build Failed!${Color_Off}"
	exit 1
fi

setEnvVars # set the necessary environment variables

cdScriptsDir
cd .. # now we are in the workspace directory
echo
echo

concurrently\
	--kill-others\
	--names "LVZ,EVZ,MAP,AV ,TG ,MI ,SI ,TI ,EV "\
	--prefix "[{name}-{time}]"\
	-c "bgWhite.black,bgWhite.black,bgGrey.inverse,bgBlue,bgRed,bgMagenta,bgBlueBright.black,bgRedBright.black,bgGreen.black"\
	--timestamp-format "HH:mm:ss"\
	"ros2 launch rt_bi_core rviz.launch.py"\
	"ros2 launch rt_bi_eventifier rviz.launch.py"\
	"ros2 launch rt_bi_emulator map.launch.py"\
	"ros2 launch rt_bi_emulator c2.avs.launch.py"\
	"ros2 launch rt_bi_emulator c2.targets.launch.py"\
	"ros2 launch rt_bi_core mi.launch.py"\
	"ros2 launch rt_bi_core si.launch.py"\
	"ros2 launch rt_bi_eventifier ev.launch.py"\
	--hide "0,1,2,3,4,5,7"
	# --hide "0,1,2"
exit 0
