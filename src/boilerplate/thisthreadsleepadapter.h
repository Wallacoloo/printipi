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


#ifndef BOILERPLATE_THISTHREADSLEEPADAPTER_H
#define BOILERPLATE_THISTHREADSLEEPADAPTER_H

#include <chrono>

//allows for sleeping to an absolute time when the custom clock (EventClockT) has a different offset than the system clock (which is otherwise used for measuring sleep times).
//@ClockT the clock to which absolute times should be compared.
//@SleepT a type or namespace that implements the sleep_for function (like ThisThreadSleepPosix or std::this_thread) which can sleep for a RELATIVE TIME.
template <typename ClockT, typename SleepT> struct ThisThreadSleepAdapter {
    template<class Clock, class Duration> static void sleep_until(const std::chrono::time_point<Clock, Duration> &sleep_time) {
        auto now = ClockT::now();
        auto rel = sleep_time.time_since_epoch() - now.time_since_epoch();
        sleep_for(rel);
    }
    template <class Rep, class Period> static void sleep_for(const std::chrono::duration<Rep, Period> &dur) {
        SleepT::sleep_for(dur);
    }
};

#endif
