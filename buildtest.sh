#!/bin/bash
#This script tests a few different build settings to make sure nothing's broken
#This script is called from .travis.yml after each push to github.
set -e #exit script if any of the commands error
set -x #echo each command

pushd src
make clean
#things to test:
#Multiple machines
#at least g++-4.6, g++-4.7, clang++-3.4, clang++-3.5
#must test both release & debug builds,
#as sometimes having the NDEBUG flag present alters the code path
for compiler in "clang++" "g++-4.6" "g++-4.7"
do
	for machine in "generic/cartesian.h" "rpi/kosselrampsfd.h"
	do
		for target in "debug" "release"
		do
			#check for compilation with and without tests enabled
			make CXX=$compiler MACHINE=$machine $target
			make CXX=$compiler MACHINE=$machine $target DO_TESTS=1
			if [ "$machine" == "generic/cartesian.h" ]; then
				#if we have a generic machine, then run the tests.
				#Cannot run platform-specific tests on all platforms.
				valgrind --leak-check=full --track-fds=yes --error-exitcode=1 ../build/printipi
		    fi
		done
	done
done
popd