#!/bin/sh
socat -d -d pty,raw,echo=0,link=./tty3dpm pty,raw,echo=0,link=./tty3dps &
sudo ln -s $(pwd)/tty3dpm /dev/ttyUSB3dp
./kossel-firmware ./tty3dps
