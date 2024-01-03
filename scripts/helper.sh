#!/bin/bash
scriptDir=$(dirname "$0")

help () {
	echo "Usage:"
	echo "\t -c | --clean       Clean build."
	echo "\t -s | --symlink     Clean, then symlink install."
	echo "\t -n | --no-build    Do not run the build script."
	echo "\t -h | -? | --help   Display this message."
}

cdScriptsDir () {
	if [ "$PWD" != "$scriptDir" ]; then cd "$scriptDir"; fi # change to the directory of the script if necessary
}

installRequirements () {
	. ~/.colors.sh
	if ! which node > /dev/null; then
		echo "${Red}Nodejs is not installed.${Color_Off}"
		exit 1
	fi

	if ! which concurrently > /dev/null; then npm install -g concurrently
	fi

	echo "${Green}Requirements are installed.${Color_Off}"
}

setEnvVars () {
	export RCUTILS_COLORIZED_OUTPUT=1
	export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
}

parseArgs () {
	##########################
	### EXIT CODES ###########
	##########################
	# 0 = No argument
	# 1 = Unknown
	# 2 = Clean Build
	# 4 = Symlink
	# 8 = No build.
	##########################
	scriptResult=0
	while [ "$#" -gt 0 ]; do
		case $1 in
			-c|--clean)
				echo "Clean build..."
				scriptResult=2
				shift ;;
			-s|--symlink)
				echo "Symlink build..."
				scriptResult=4
				shift ;;
			-n|--no-build)
				echo "No build..."
				scriptResult=8
				shift ;;
			*)
				echo "Unknown arg: $1"
				scriptResult=1
				shift ;;
		esac
	done
	return $scriptResult
}

sourceWorkspace () {
	cdScriptsDir
	cd .. # now we are in the workspace directory
	echo "source $PWD/install/local_setup.sh"
	# https://stackoverflow.com/a/13702876/750567
	. $PWD/install/local_setup.sh
	if [ $? -eq 0 ]
	then
		echo "Workspace sourced."
	else
		echo "Workspace source failed!"
		exit 1
	fi
}

printEnv () {
	printenv
}
