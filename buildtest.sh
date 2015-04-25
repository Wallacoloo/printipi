#!/bin/bash
#This script tests a few different build settings to make sure nothing's broken
#This script is called from .travis.yml after each push to github.
set -e #exit script if any of the commands error
set -x #echo each command

#If running in Travis CI, add path to valgrind binary:
PATH=$PATH:/home/travis/.local/bin

pushd src
make clean
#things to test:
#Multiple machines
#at least g++-4.6, g++-4.7, clang++-3.4
#must test both release & debug/debugrel builds,
#as sometimes having the NDEBUG flag present alters the code path
#Must test g++-4.7 in addition to g++-4.6, as the later uses LTO which can introduce failed compilations
for compiler in "clang++" "g++-4.6" "g++-4.7"
do
	for machine in machines/*/*.h
	do
		for target in "debugrel" "release"
		do
			#check for compilation against the native platform and the generic platform
			make CXX=$compiler MACHINE=$machine $target -j2
			make CXX=$compiler MACHINE=$machine $target PLATFORM=generic ENABLE_TESTS=1 -j2
			#only run valgrind on clang builds to avoid gcc generating instructions valgrind doesn't recognize
			if [ "$compiler" == "clang++" ] ; then 
				valgrind --leak-check=full --track-fds=yes --error-exitcode=1 ../build/printipi --do-tests
			else
				../build/printipi --do-tests
			fi
		done
	done
done
popd