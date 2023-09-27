#!/bin/bash
echo
. $(dirname "$0")/helper.sh
installRequirements
if [ ! -e $PWD/install/local_setup.sh ]
then
	/bin/bash $PWD/build.sh
fi
source_workspace
cdScriptsDir # change to the directory of the script
echo
echo

python3 $PWD/main.utils.py "$@" # pass-thru the arguments
exit $?
