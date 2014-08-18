Printipi
========

Printipi is a software package designed to bring 3d printing to the Raspberry Pi. It takes on all of the roles generally given to dedicated microcontrollers (interfacing with stepper drivers, temperature control of the hotend, and cooling fans) while also running under an operating system. This means that the same device that is running the firmware can also perform other tasks while printing, such as hosting a web interface like Octoprint.

Although called Printipi, it is capable of running on other boards than the Raspberry Pi provided that the compiler supports C++11. One need only reproduce the functionality of the 25 lines contained in src/drivers/rpi/rpiiopin.h (which explains how to read or write an IO pin) for their device.

Printipi also aims to support a multitude of printers including typical cartesian printers, delta-style printers like the Kossel, or polar-based printers - **without** the messy use of hundreds of #defines, some of which may not even be applicable to your printer. Instead, each machine type gets its own file and C++ class under src/drivers/machines that exposes its coordinate system and peripherals through a handful of public member functions and typedefs. In this way it is possible to add support for a new type of printer without digging into the guts of Printipi.

Demos
========

Printipi powering a Mini Kossel: http://youtu.be/g4UD5MRas3E  

License
========

With the exception of certain files*, Printipi is licensed under the MIT license. This means that you are free to use, modify, distribute, and sublicense the code as you see fit. While you are not obligated to do so by the license, it would be appreciated that you share any improvements you make (eg make a public fork on github containing your modifications and then submit a pull request to have it merged with the master branch).

*firmware/src/drivers/rpi/bcm2835.c and firmware/src/drivers/rpi/bcm2835.h are (c) Mike McCauley  
*util/rotation_matrix.py is (c) Edward d'Auvergne and provided here only for convenience

Limitations
========

Printipi currently runs entirely in userland. While this makes development and usage trivial, timing suffers. By running under a high priority & locking memory to prevent page-swaps, it can still print successfully. Effort has also been made towards recovering from missed steps. A kossel-style printer can currently move at about 90 mm/sec @ 1/4 stepping without skipping (and using about 75% cpu).

Currently, only a limited set of gcode commands are supported. Namely, testing has been done using Cura for slicing. Furthermore, all comments must first be stripped from the input.

Compiling
========

**Prereqs**: gcc >= 4.6 or another compiler with support for C++11  
gcc >= 4.7 is highly recommended because of the benefits gained from link-time optimization, which isn't supported in gcc 4.6 when using C++11  
gcc 4.7 can be installed in the stock version of Raspbian via `sudo apt-get install g++-4.7` and to use it over the system's gcc, compile with `make CXX=g++-4.7`

First, get the sources: `git clone --recursive https://github.com/Wallacoloo/printipi.git`  
The recursive flag is required due to the use of git submodules inside the project.

To compile Printipi, navigate to code/firmware/src and type `make MACHINE=<machine> <target>`, where `<machine>` is the C++ classname of the machine contained under src/drivers/machines, eg `KosselPi` or the `Example` machine, and `<target>` is either debug, release, profile, or minsize. Both are case-sensitive. A binary will be produced under code/firmware/build with the same name as your machine. Navigate to that folder and run the binary (you will want root permissions in order to elevate the priority of the task, so run eg 'sudo ./kosselpi'.

Usage
========

The firmware can either be called with no arguments, in which case it will take gcode commands from the standard input (useful for testing & debugging). Or, you can provide the path to a gcode file (note that gcode parsing is currently limited. Comments aren't understood, for example). The provided file can be **any** file-like object, including device-files. This allows one to pass eg /dev/ttyAMA0 to take commands from the serial port.

Using with Octoprint:
--------

**Prereqs**: install the program "socat". Eg `sudo apt-get install socat`

Because Octoprint prints to a serial-like Linux device-file, and Printipi can take commands from any file-like object, it's possible to create a *virtual* serial port to pipe commands from Octoprint to Printipi. This is just what the provided "launch-firmware.sh" file does in the firmware device. After running that script, a new device should be visible in the Octoprint web interface (a refresh will be required) to which you can connect. 

The Future
========

The short-to-midterm goals for Printipi are mostly optimization-based. Although running in userland is nice, it would be better to move at least the timing-critical sections into a kernel module. It may also be possible to make use of the DMA channels in the Raspberry Pi to achieve greater step-rates and sub-microsecond precise timings.

More effort will also be put into the motion planning system, which currently has no concept of curves and thus forces a full deceleration to 0 at each joint in the path.

Lastly, it will be necessary to make the gcode parser properly handle transmission errors.
