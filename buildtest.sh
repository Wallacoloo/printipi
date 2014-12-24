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
			make CXX=$compiler MACHINE=$machine $target
		done
	done
done
popd