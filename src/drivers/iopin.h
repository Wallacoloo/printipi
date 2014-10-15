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
 * Printipi/drivers/iopin.h
 *
 * IoPin defines the interface for a GPIO pin, as well as default implementations of each function in case they aren't supported by the actual driver. 
 * Each microcontroller platform should provide its own IoPin implementation that inherits from this class.
 */

#ifndef DRIVERS_IOPIN_H
#define DRIVERS_IOPIN_H

#include "common/logging.h"
#include "common/typesettings/primitives.h" //for GpioPinIdType

namespace drv {

enum IoLevel { //TODO: may be simpler to just move everything to bools, where 1 == High.
    IoLow = 0,
    IoHigh = 1
};

template <typename ThisT, IoLevel lev=IoLow> class IoPinOnExit {
    //Often times it is useful to ensure that a given pin is put into a given state upon program exit.
    //When this class is instantiated, it inserts a hook into the exit handlers to do just that.
    static void deactivate() {
        LOGV("IoPinOnExit::deactivate\n");
        ThisT pin;
        pin.digitalWrite(lev);
    }
    public:
        IoPinOnExit() {
            LOGV("IoPinOnExit init\n");
            SchedulerBase::registerExitHandler((void(*)())&deactivate, SCHED_IO_EXIT_LEVEL);
        }
};

struct IoPin {
    //IoPin defines the interface for a GPIO pin, as well as default implementations of each function in case they aren't supported by the actual driver. 
    //Each microcontroller platform should provide its own IoPin implementation that inherits from this class.
    //Each pin has a physical Id, which can be used for some forms of hardware acceleration.
    inline GpioPinIdType id() const { return -1; }
    //configure the pin as an output. Many microcontrollers have GPIOs. Before writing an output value to them, they must first be configured as an output. But we also want the initial state of the pin to be defined, if possible.
    inline void makeDigitalOutput(IoLevel) {}
    //configure the pin to be an input
    inline void makeDigitalInput() {}
    //read the pin's input value (assumes pin is configured as digital)
    inline IoLevel digitalRead() const { return IoLow; }
    inline void digitalWrite(IoLevel) {}
    inline bool areWritesInverted() const { return false; }
};

//easy way to proxy reads/writes, but invert them (eg Inverted<IoPin>.digitalWrite(IoHigh) really calls IoPin.digitalWrite(IoLow), etc)
template <typename Pin, bool InvertWrite=true, bool InvertRead=true> class InvertedPin : public IoPin {
    Pin _pin;
    IoLevel invertLev(IoLevel lev, bool doInvert) const {
        return doInvert ? (lev == IoHigh ? IoLow : IoHigh) : lev;
    }
    public:
        void makeDigitalOutput(IoLevel lev) {
            _pin.makeDigitalOutput(invertLev(lev, InvertWrite));
        }
        inline void makeDigitalInput() {
            _pin.makeDigitalInput();
        }
        inline IoLevel digitalRead() const {
            return invertLev(_pin.digitalRead(), InvertRead);
        }
        inline void digitalWrite(IoLevel lev) {
            _pin.digitalWrite(invertLev(lev, InvertWrite));
        }
        inline bool areWritesInverted() const {
            return !_pin.areWritesInverted();
        }
};

//default implementation of IoPin (does nothing):
struct NoPin : public IoPin { };

}
#endif
