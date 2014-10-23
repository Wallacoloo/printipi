#ifndef COMMON_TYPESETTINGS_COMPILEFLAGS_H
#define COMMON_TYPESETTINGS_COMPILEFLAGS_H

//allow for generation of code that still works in high-latency enviroments, like valgrind
#ifdef DRUNNING_IN_VM
    #define RUNNING_IN_VM 1
#else
    #define RUNNING_IN_VM 0
#endif

#ifdef DTARGET_PLATFORM_RPI
    #define TARGET_RPI //provide a user-usable macro
#endif

//pthread isn't required, but can provide higher-elevated thread priority
#define USE_PTHREAD DUSE_PTHREAD

#ifdef DNO_DMA
    #define NO_DMA
#endif


#endif
