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

#ifndef COMMON_INTERVALTIMER_H
#define COMMON_INTERVALTIMER_H

#include "platforms/auto/chronoclock.h" //for EventClockT


/* 
 * IntervalTimer provides a way to clock the time between two events (or between calls to a recurring event).
 * This can be used to detect when an input isn't being serviced regularaly enough (eg in src/drivers/tempcontrol.h)
 */
class IntervalTimer {
    EventClockT::time_point _last;
    public:
        inline IntervalTimer() : _last() {}
        inline void reset() {
            _last = EventClockT::time_point();
        }
        inline const EventClockT::time_point& clock() {
            return _last = EventClockT::now();
        }
        inline const EventClockT::time_point& get() const {
            return _last;
        }
        inline EventClockT::duration clockDiff() {
            EventClockT::time_point next = EventClockT::now();
            EventClockT::duration diff = next-_last;
            _last = next;
            return diff;
        }
        template <typename DurT> int clockCmp(const DurT &cmp, int dflt=0) {
            int ret;
            EventClockT::time_point now = EventClockT::now();
            if (_last == EventClockT::time_point()) { //no last time
                ret = dflt;
            } else {
                auto duration = now - _last;
                ret = duration > cmp ? 1 : (duration < cmp ? -1 : 0);
            }
            _last = now;
            return ret;
        }
};



#endif
