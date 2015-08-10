#!/bin/sh
sudo rm -f /dev/ttyUSBprintipi
socat -d -d pty,raw,echo=0,link=./ttyprintipi-master pty,raw,echo=0,link=./ttyprintipi-slave &
sudo ln -s $(pwd)/ttyprintipi-master /dev/ttyUSBprintipi
#Octoprint needs to be able to read from its tty:
sudo chmod +r ./ttyprintipi-master ./ttyprintipi-slave

echo "Launching Printipi firmware configured to take commands from /dev/ttyUSBprintipi"
echo "NOTE: This will appear to hang the console, but that is not an error"
echo "Killing the console (Ctrl+c or Ctrl+z) will also kill Printipi & leave /dev/ttyUSBprintipi unresponsive"
#need sudo for io control. #nice --15. Negative nice values = higher priority. nice -20 is low pri, nice --20 is high pri
sudo nice --15 ./build/printipi ./ttyprintipi-slave ./ttyprintipi-slave $@
