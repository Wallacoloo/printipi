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

#ifndef PLATFORMS_AUTO_CHRONOCLOCK_H
#define PLATFORMS_AUTO_CHRONOCLOCK_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_CHRONOCLOCK
    #include PLATFORM_DRIVER_CHRONOCLOCK
    typedef plat::TARGET_PLATFORM_LOWER::ChronoClock EventClockT;
#else
    #ifdef COMPILING_MAIN
        #warning "using ChronoClockPosix for EventClockT. While this does work, you will get better performance if you use a clock specific to your machine (implement src/platforms/<PLATFORM>/chronoclock.h)"
    #endif
    #include "platforms/generic/chronoclock.h"
    typedef plat::generic::ChronoClock EventClockT;
#endif

#endif
