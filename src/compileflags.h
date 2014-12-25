#ifndef TYPESETTINGS_COMPILEFLAGS_H
#define TYPESETTINGS_COMPILEFLAGS_H

#include <cstdint> //for uint8_t


//The BCM2835 chip has 54 pins (#0-53).
//Only the lower 32 are usable on the 3 current models: A, B, and A+/B+
//This allows for some code optimizations, but we want to somewhat future-proof them in case a new model adds more IO:
#ifndef MAX_RPI_PIN_ID
    #define MAX_RPI_PIN_ID 31
#endif

//allow for generation of code that still works in high-latency enviroments, like valgrind
#ifdef DRUNNING_IN_VM
    #define RUNNING_IN_VM 1
#else
    #define RUNNING_IN_VM 0
#endif

//e.g. TARGET_PLATFORM_LOWER="rpi" or "generic"
#define TARGET_PLATFORM_LOWER DTARGET_PLATFORM_LOWER

//pthread isn't required, but can provide higher-elevated thread priority
#ifdef DUSE_PTHREAD
	#define USE_PTHREAD 1
#else
	#define USE_PTHREAD 0
#endif

#ifdef DNO_DMA
    #define NO_DMA
#endif

#ifdef BUILD_TYPE_release
	//on a debug build, we might want to let exceptions raise all the way up into the OS
	//but on a release build, we want to catch them and display our own error message.
    #define CLEAN_EXIT
#endif

#ifdef DDO_TESTS
	#define DO_TESTS 1
#else
	#define DO_TESTS 0
#endif


//Now expose some primitive typedefs:
typedef uint8_t AxisIdType;
typedef float CelciusType;
typedef bool IoLevel;
#define IoLow false
#define IoHigh true
#define IoDefaultLow IoLow
#define IoDefaultHigh IoHigh
#define IoDefaultOpenCircuit IoDefaultLow //TODO: misleading! make an open-circuit (high-impedance input) default actually possible.

#endif
