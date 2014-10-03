#ifndef COMMON_TYPESETTINGS_COMPILEFLAGS_H
#define COMMON_TYPESETTINGS_COMPILEFLAGS_H

//allow for generation of code that still works in high-latency enviroments, like valgrind
#ifdef DRUNNING_IN_VM
    #define RUNNING_IN_VM 1
#else
    #define RUNNING_IN_VM 0
#endif

#if DTARGET_RPI == 1
    #define TARGET_RPI //provide a user-usable macro
#endif

#define USE_PTHREADS DUSE_PTHREADS


#endif
