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
 
/*
 * Printipi/platforms/rpi/chronoclockrpi.h
 *
 * Implements the same interfaces as std::chrono::<clock>::now()
 *   to allow for highly efficient access to the system time (no context switching to kernel)
 *
 * NOTE: The time reported by ChronoClockRpi may be different than the actual system time, so make sure to use consistent clock sources!
 */
 
#ifndef PLATFORMS_RPI_CHRONOCLOCK_H
#define PLATFORMS_RPI_CHRONOCLOCK_H

#include <chrono> //for std::chrono::*
#include "mitpi.h"


namespace plat {
namespace rpi {


class ChronoClock {
    //special implementation of std::chrono::clock<...>
    static mitpi::InitMitpiType _i; //ensure mitpi is init before any calls to now() occur.
    public:
        typedef std::chrono::microseconds duration;
        typedef duration::rep rep;
        typedef duration::period period;
        typedef std::chrono::time_point<ChronoClock> time_point;
        static const bool is_steady = true;
        inline static time_point now() noexcept {
            //struct timespec tnow = timespecNow();
            //return time_point(std::chrono::microseconds(bcm2835_st_read()));
            return time_point(std::chrono::microseconds(mitpi::readSysTime()));
        }
};


}
}

#endif
