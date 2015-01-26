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

#ifndef PLATFORMS_AUTO_THISTHREADSLEEP_H
#define PLATFORMS_AUTO_THISTHREADSLEEP_H

#include "compileflags.h"

#ifdef PLATFORM_DRIVER_THISTHREADSLEEP
    #include PLATFORM_DRIVER_THISTHREADSLEEP
    typedef plat::TARGET_PLATFORM_LOWER::ThisThreadSleep SleepT;
#else

    #ifdef PLATFORM_DRIVER_CHRONOCLOCK
        //custom platform clock type. Must make ALL sleeps relative (unless platform also provides ThisThreadSleep
        #include "boilerplate/thisthreadsleepadapter.h"
        #include "platforms/generic/thisthreadsleep.h"
        typedef ThisThreadSleepAdapter<EventClockT, plat::generic::ThisThreadSleep> SleepT;
    #else
        //generic ChronoClock, so can use generic ThisThreadSleep
        #include "platforms/generic/thisthreadsleep.h"
        typedef plat::generic::ThisThreadSleep SleepT;
    #endif
    
#endif

#endif
