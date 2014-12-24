#!/bin/bash

set -e #exit script if any of the commands error

./pushdoc.sh
./buildtest.sh
