#!/bin/bash
pushd src
sudo apt-get install -qq g++-4.6 g++-4.7
make CXX=g++-4.6 MACHINE=rpi/kosselrampsfd.h
make CXX=g++-4.6 MACHINE=generic/example.h
make CXX=g++-4.7 MACHINE=rpi/kosselrampsfd.h
make CXX=g++-4.7 MACHINE=generic/example.h
popd