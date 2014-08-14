Printipi
========

Printipi is a software package designed to bring 3d printing to the Raspberry Pi. It takes on all of the roles generally given to dedicated microcontrollers (interfacing with stepper drivers, temperature control of the hotend, and cooling fans) while also running under an operating system. This means that the same device that is running the firmware can also perform other tasks while printing, such as hosting a web interface like Octoprint.

Although called Printipi, it is capable of running on other boards than the Raspberry Pi provided that the compiler supports C++11. One need only reproduce the functionality of the 25 lines contained in src/drivers/rpi/rpiiopin.h (which explains how to read or write an IO pin) for their device.

Printipi also aims to support a multitude of printers including typical cartesian printers, delta-style printers like the Kossel, or polar-based printers - **without** the messy use of #defines. Instead, each machine type gets its own file under src/drivers/machines and exposes its coordinate system and peripherals through public typedefs.

Limitations
========

Printipi currently runs entirely in userland. While this makes development and usage trivial, timing suffers. By running under a high priority & locking memory to prevent page-swaps, it can still print successfully. Effort has also been made towards recovering from missed steps. A kossel-style printer can currently move at about 90 mm/sec @ 1/4 stepping without skipping (and using about 75% cpu).

The Future
========

The short-to-midterm goals for Printipi are mostly optimization-based. Although running in userland is nice, it would be better to move at least the timing-critical sections into a kernel module. It may also be possible to make use of the DMA channels in the Raspberry Pi to achieve greater step-rates and sub-microsecond precise timings.

More effort will also be put into the motion planning system, which currently has no concept of curves and thus forces a full deceleration to 0 at each join in the path.
