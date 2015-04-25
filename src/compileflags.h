/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

#ifdef DNO_LOGGING
    #define DO_LOG 0
#else
    #define DO_LOG 1
#endif
#ifdef DNO_LOG_M105
    #define NO_LOG_M105 1
#else
    #define NO_LOG_M105 0
#endif

#ifdef BUILD_TYPE_release
	//on a debug build, we might want to let exceptions raise all the way up into the OS
	//but on a release build, we want to catch them and display our own error message.
    #define CLEAN_EXIT 1
#else
    #define CLEAN_EXIT 0
#endif

#ifdef DENABLE_TESTS
	#define ENABLE_TESTS 1
#else
	#define ENABLE_TESTS 0
#endif


//Now expose some primitive typedefs:

//This determines the maximum number of axis we can have on the machine
//
// Since most machines only need 3-6 degrees of freedom, an 8-bit integer is PLENTY
typedef uint8_t AxisIdType;
typedef float CelciusType;
typedef bool IoLevel;
#define IoLow false
#define IoHigh true

#endif
