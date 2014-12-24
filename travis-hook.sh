#!/bin/bash

set -e #exit script if any of the commands error
set -x #echo each command

./pushdoc.sh
./buildtest.sh
