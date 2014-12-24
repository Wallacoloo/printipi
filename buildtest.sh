#!/bin/bash
#This script tests a few different build settings to make sure nothing's broken
#This script is called from .travis.yml after each push to github.
set -e #exit script if any of the commands error
pushd src
make clean
#things to test:
#Multiple machines
#at least g++-4.6, g++-4.7
#must test both release & debug builds,
#as sometimes having the NDEBUG flag present alters the code path
make CXX=g++-4.6 MACHINE=rpi/kosselrampsfd.h debug
make CXX=g++-4.6 MACHINE=rpi/kosselrampsfd.h release
make CXX=g++-4.6 MACHINE=generic/cartesian.h debug
make CXX=g++-4.6 MACHINE=generic/cartesian.h release
make CXX=g++-4.7 MACHINE=rpi/kosselrampsfd.h debug
make CXX=g++-4.7 MACHINE=rpi/kosselrampsfd.h release
make CXX=g++-4.7 MACHINE=generic/cartesian.h debug
make CXX=g++-4.7 MACHINE=generic/cartesian.h release

make CXX=clang++-3.4 MACHINE=rpi/kosselrampsfd.h debug
make CXX=clang++-3.4 MACHINE=rpi/kosselrampsfd.h release
make CXX=clang++-3.4 MACHINE=generic/cartesian.h debug
make CXX=clang++-3.4 MACHINE=generic/cartesian.h release
make CXX=clang++-3.5 MACHINE=rpi/kosselrampsfd.h debug
make CXX=clang++-3.5 MACHINE=rpi/kosselrampsfd.h release
make CXX=clang++-3.5 MACHINE=generic/cartesian.h debug
make CXX=clang++-3.5 MACHINE=generic/cartesian.h release
popd