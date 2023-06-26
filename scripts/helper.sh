#!/bin/bash

help () {
	echo "Usage:"
	echo "\t -c | --clean       Clean build."
	echo "\t -s | --symlink     Clean, then symlink install."
	echo "\t -n | --no-build    Do not run the build script."
	echo "\t -h | -? | --help   Display this message."
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
				scriptResult|=2
				shift ;;
			-s|--symlink)
				echo "Symlink build..."
				scriptResult|=4
				shift ;;
			-n|--no-build)
				echo "No build..."
				scriptResult|=8
				shift ;;
			*)
				echo "Unknown arg: $1"
				scriptResult=1
				shift ;;
		esac
	done
	return $scriptResult
}
