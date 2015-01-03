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

#ifndef PLATFORMS_GENERIC_CHRONOCLOCK
#define PLATFORMS_GENERIC_CHRONOCLOCK

/*
 * gcc 4.6 doesn't support std::chrono::clock stuff.
 * gcc 4.7 on arm implements it incorrectly (later versions untested)
 * Therefore, we implement our own clock that has the same interface which we can use within Linux (if on unix or apple).
 *   Otherwise, we default to std::chrono and hope for the best.
*/

#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
    #include <chrono>
    #include <time.h>
    namespace plat {
    namespace generic {
        class ChronoClock {
            public:
                typedef std::chrono::nanoseconds duration;
                typedef duration::rep rep;
                typedef duration::period period;
                typedef std::chrono::time_point<ChronoClock> time_point;
                static const bool is_steady = true;
                inline static time_point now() noexcept {
                    struct timespec tnow;
                    clock_gettime(CLOCK_MONOTONIC, &tnow);
                    return time_point(std::chrono::seconds(tnow.tv_sec) + std::chrono::nanoseconds(tnow.tv_nsec));
                }
        };
    }
    }
#else //default to C++11 clocks and hope for the best:
    #include <chrono>
    namespace plat {
    namespace generic {
        typedef std::chrono::steady_clock EventClockT;
    }
    }
#endif

#endif
