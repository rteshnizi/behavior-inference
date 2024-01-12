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
	--kill-others\
	--names "ESE,ESY,ETG,IMP,ISE,ISY,SMP,EVZ,EV ,EVZ,RUN"\
	--prefix "[{name}-{time}]"\
	-c "bgGreen,bgBlue,bgRed,bgWhite.black.underline,bgGreen.inverse,bgBlue.inverse,bgWhite.black,bgYellow.black,bgMagenta,bgYellow.black,bgCyan.black"\
	--timestamp-format "HH:mm:ss"\
	"ros2 launch rt_bi_emulator c2.ese.launch.py"\
	"ros2 launch rt_bi_emulator c2.esy.launch.py"\
	"ros2 launch rt_bi_emulator c2.etg.launch.py"\
	"ros2 launch rt_bi_emulator imp.launch.py"\
	"ros2 launch rt_bi_emulator ise.launch.py"\
	"ros2 launch rt_bi_emulator isy.launch.py"\
	"ros2 launch rt_bi_emulator smp.launch.py"\
	"ros2 launch rt_bi_emulator rviz.launch.py"\
	"ros2 launch rt_bi_eventifier ev.launch.py"\
	"ros2 launch rt_bi_eventifier rviz.launch.py"\
	"ros2 launch rt_bi_runtime c2.all.launch.py"\
	--hide "0,1,2,3,4,6,7,8,9,10"
exit 0
