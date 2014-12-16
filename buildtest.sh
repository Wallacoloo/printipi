#!/bin/bash
#This script tests a few different build settings to make sure nothing's broken
#This script is called from .travis.yml after each push to github.
set -e #exit script if any of the commands error
pushd src
make clean
make CXX=g++-4.6 MACHINE=rpi/kosselrampsfd.h
make CXX=g++-4.6 MACHINE=generic/cartesian.h
make CXX=g++-4.7 MACHINE=rpi/kosselrampsfd.h
make CXX=g++-4.7 MACHINE=generic/cartesian.h
popd