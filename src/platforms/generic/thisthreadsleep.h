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

#ifndef PLATFORMS_GENERIC_THISTHREADSLEEP_H
#define PLATFORMS_GENERIC_THISTHREADSLEEP_H

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))

    #include <chrono>
    #include <time.h>

    namespace plat {
    namespace generic {

    /*
     * std::this_thread::sleep_until may be having issues when using custom clocks. Try this as a workaround if using Posix.
    */
    class ThisThreadSleep {
        public:
            template<class Clock, class Duration> static void sleep_until(const std::chrono::time_point<Clock, Duration> &sleep_time) {
                auto dur = sleep_time.time_since_epoch();
                auto durSec = std::chrono::duration_cast<std::chrono::seconds>(dur);
                auto durNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur) - durSec;
                timespec tsSleepUntil;
                tsSleepUntil.tv_sec = durSec.count();
                tsSleepUntil.tv_nsec = durNsec.count();
                clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &tsSleepUntil, nullptr);
            }
            template <class Rep, class Period> static void sleep_for(const std::chrono::duration<Rep, Period> &dur) {
                auto durSec = std::chrono::duration_cast<std::chrono::seconds>(dur);
                auto durNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(dur) - durSec;
                timespec tsSleepUntil;
                tsSleepUntil.tv_sec = durSec.count();
                tsSleepUntil.tv_nsec = durNsec.count();
                clock_nanosleep(CLOCK_MONOTONIC, 0, &tsSleepUntil, nullptr); //0 = time given is relative.
            }
    };

    }
    }
    
#else 
    //default to C++11 thread interface and hope for the best:
    #include <thread>
    namespace plat {
    namespace generic {
        typedef std::this_thead SleepT;
    }
    }

#endif

#endif
