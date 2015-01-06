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
 
#ifndef OUTPUTEVENT_H
#define OUTPUTEVENT_H
 
#include "platforms/auto/chronoclock.h" //for EventClockT
#include "platforms/auto/primitiveiopin.h"
#include "iodrivers/iopin.h"
#include "compileflags.h" //for IoLevel

/* 
 * An OutputEvent encapsulates information about the desired state for a GPIO pin at a given time.
 *   Eg, "set pin (2) (high) at t=(1234567) uS", or "set pin (44) (low) at t=(887766) uS"
 */
class OutputEvent {
    EventClockT::time_point _time;
    PrimitiveIoPin _pin;
    IoLevel _state;
    public:
        //default constructor. Creates an OutputEvent where isNull() will return true
        inline OutputEvent()
        : _time(std::chrono::seconds(0)), _pin(PrimitiveIoPin::null()), _state(IoLow) {
        }
        //Construct from a time point, a pin and a state.
        //@time the time at which the pin state should be altered.
        //@state the logical state the pin should be put in *before* any inversions have been processed.
        //  so if @state is IoHigh, pin.areWritesInverted() == true, then the physical hardware pin will output LOW (0V / Ground)
        inline OutputEvent(EventClockT::time_point time, const iodrv::IoPin &pin, bool state) 
        : _time(time), _pin(pin.primitiveIoPin()), _state(pin.translateWriteToPrimitive(state)) {
        }
        //@return the time at which the pin state should be altered.
        inline EventClockT::time_point time() const {
            return _time;
        }
        //@return the pin that should be affected.
        inline const PrimitiveIoPin& primitiveIoPin() const {
            return _pin;
        }
        //@return the pin that should be affected.
        inline PrimitiveIoPin& primitiveIoPin() {
            return _pin;
        }
        //@return the state that the pin should be set to.
        //  Note: since this is a PrimitiveIoPin, any inversions have already been processed.
        //    so if state() returns IoHigh, then the pin should output 3.3v (or 5v, etc).
        //    if IoLow, then pin should output 0v (ground)
        inline bool state() const {
            return _state;
        }
        //@return true if the OutputEvent was default-constructed.
        inline bool isNull() const {
            return _time == EventClockT::time_point(std::chrono::seconds(0));
        }
};


#endif
