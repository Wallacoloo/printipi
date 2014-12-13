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
 * Printipi/outputevent.h
 *
 * An OutputEvent encapsulates information about the desired state for a GPIO pin at a given time.
 *   Eg, "set pin (2) (high) at t=(1234567) uS", or "set pin (44) (low) at t=(887766) uS"
 */
 
#ifndef OUTPUTEVENT_H
#define OUTPUTEVENT_H
 
#include "drivers/auto/chronoclock.h" //for EventClockT
#include "compileflags.h" //for GpioPinIdType

class OutputEvent {
    EventClockT::time_point _time;
    GpioPinIdType _pinId;
    bool _state; //1=HIGH, 0=LOW
    public:
        inline OutputEvent(EventClockT::time_point time, GpioPinIdType pinId, bool state) : _time(time), _pinId(pinId), _state(state) {
        }
        inline EventClockT::time_point time() const {
            return _time;
        }
        inline GpioPinIdType pinId() const {
            return _pinId;
        }
        inline bool state() const {
            return _state;
        }
};


#endif
