#!/bin/sh
unbuffer bash launch-firmware.sh --verbose | tee log.txt
