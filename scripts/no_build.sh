#!/bin/bash
echo scripts/no_build.sh

. $(dirname "$0")/helper.sh
source_workspace

concurrently\
	--kill-others\
	--names "RVZ,MAP,AV ,MI ,SI "\
	--prefix "[{name}-{time}]"\
	-c "bgWhite.black,bgBlueBright.black,bgBlue.black,bgYellow.black,bgMagentaBright.black"\
	--timestamp-format "HH:mm:ss"\
	"ros2 launch rt_bi_core rviz.launch.py"\
	"ros2 launch rt_bi_emulator map.launch.py"\
	"ros2 launch rt_bi_emulator av1.2.launch.py"\
	"ros2 launch rt_bi_core mi.launch.py"\
	"ros2 launch rt_bi_core si.launch.py"\
	--hide "0,1,3,4"
	# --hide "0,1,2,3,4"

echo scripts/no_build.sh done

exit 0
