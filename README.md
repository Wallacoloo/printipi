Printipi
========

Printipi is a software package designed to bring 3d printing to the Raspberry Pi. It takes on all of the roles generally given to dedicated microcontrollers (interfacing with stepper drivers, temperature control of the hotend, and cooling fans) while also running under an operating system. This means that the same device that is running the firmware can also perform other tasks while printing, such as hosting a web interface like Octoprint.

Although called Printipi, it is not necessarily limited to running on the Pi. The `Example` machine can compile and run on most Linux machines, as a proof of concept (it does no electrical I/O), and new machines can be supported by implementing a handful of interfaces (see the section below for more info).

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

Printipi currently runs entirely in userland. While this makes development and usage trivial, it makes hardware management less safe. Printipi uses one of the Raspberry Pi's DMA channels in order to achieve precise output timing (2~4uS precision), however if another program tries to access the same DMA channel as Printipi, it **will** lead to errors.  
Also, very heavy bus contention may degrade timing accuracy. Experiments show 500ksamples/sec (2uS resolution) to be dependable under most operating conditions, except heavy network/disk usage. 250ksamples/sec (4uS resolution) is dependable for at least 1 MB/sec network loads, and is the default mode.

Currently, only a limited set of gcode commands are supported. Namely, testing has been done using Cura for slicing.

Compiling
========

**Prereqs**: gcc >= 4.6 or another compiler with support for C++11  
gcc >= 4.7 is highly recommended because of the benefits gained from link-time optimization, which isn't supported in gcc 4.6 when using C++11  
gcc 4.7 can be installed in the stock version of Raspbian via `sudo apt-get install g++-4.7` and to use it over the system's gcc, compile with `make CXX=g++-4.7`

First, get the sources: `git clone https://github.com/Wallacoloo/printipi.git`  

To compile Printipi, navigate to code/firmware/src and type `make MACHINE=<machine> <target>`, where `<machine>` is the C++ classname of the machine contained under src/drivers/machines, eg `KosselPi` or the `Example` machine, and `<target>` is either debug, release, profile, or minsize. Both are case-sensitive. A binary will be produced under code/firmware/build with the name `printipi`. Navigate to that folder and run the binary (you will want root permissions in order to elevate the priority of the task, so run eg `sudo ./printipi`).

Usage
========

The firmware can either be called with no arguments, in which case it will take gcode commands from the standard input (useful for testing & debugging). Or, you can provide the path to a gcode file. The provided file can be **any** file-like object, including device-files. This allows one to pass eg `/dev/ttyAMA0` to take commands from the serial port.

Using with Octoprint:
--------

**Prereqs**: install the program "socat". Eg `sudo apt-get install socat`

Because Octoprint prints to a serial-like Linux device-file, and Printipi can take commands from any file-like object, it's possible to create a *virtual* serial port to pipe commands from Octoprint to Printipi. This is just what the provided "launch-firmware.sh" file does in the firmware device. After running that script, a new device should be visible in the Octoprint web interface (a refresh will be required) to which you can connect. 

Supporting Other Architectures
========

While Printipi is under heavy development, this process may change slightly, but these are the basic steps to supporting new hardware:  
1. Implement drivers/IoPin. An example implementation is drivers/rpi/RpiIoPin  
2. Implement the HardwareScheduler interface found in schedulerbase.h and update common/typesettings/schedinterfacehardwarescheduler.h to use your implementation  
3. Make a new class for your machine in drivers/machines  

Congratulations, you're now running Printipi!

The Future
========

The short to midterm goals for Printipi are mostly optimization-based. DMA has significantly minimized timing issues, but more optimizations are needed in order to lower cpu usage & plan paths more rapidly.

More effort will also be put into the motion planning system, which currently has no concept of curves and thus forces a full deceleration to 0 at each joint in the path.

Lastly, it will be necessary to make the gcode parser properly handle transmission errors.

See the issues section for more info.
