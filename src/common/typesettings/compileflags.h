#ifndef COMMON_TYPESETTINGS_COMPILEFLAGS_H
#define COMMON_TYPESETTINGS_COMPILEFLAGS_H

//The BCM2835 chip has 54 pins (#0-53).
//Only the lower 32 are usable on the 3 current models: A, B, and B+
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

#ifdef DTARGET_PLATFORM_RPI
    #define TARGET_RPI //provide a user-usable macro
#endif
//e.g. TARGET_PLATFORM_LOWER="rpi" or "generic"
#define TARGET_PLATFORM_LOWER DTARGET_PLATFORM_LOWER

//pthread isn't required, but can provide higher-elevated thread priority
#define USE_PTHREAD DUSE_PTHREAD

#ifdef DNO_DMA
    #define NO_DMA
#endif

#endif
