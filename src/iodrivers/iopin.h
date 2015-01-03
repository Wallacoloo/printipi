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

#ifndef DRIVERS_IOPIN_H
#define DRIVERS_IOPIN_H

#include <set>
#include <utility> //for std::move
#include "platforms/auto/primitiveiopin.h"

namespace iodrv {

enum IoPinInversions {
    //bitfield that can be used to indicate whether a pin has logically-inverted reads/writes
    NO_INVERSIONS = 0,
    INVERT_READS  = 1,
    INVERT_WRITES = 2
};

enum DefaultIoState {
    IO_DEFAULT_NONE,
    IO_DEFAULT_HIGH_IMPEDANCE,
    IO_DEFAULT_LOW,
    IO_DEFAULT_HIGH
};


/*
 * IoPin defines the interface for a GPIO pin, as well as default implementations of each function in case they aren't supported by the actual driver. 
 * Each microcontroller platform should provide its own IoPin implementation that inherits from this class.
 */
class IoPin {
    PrimitiveIoPin _pin;
    bool _invertReads;
    bool _invertWrites;
    DefaultIoState _defaultState;

    static std::set<IoPin*> livingPins;

    public:
        //forward-declare a 'null' class for IoPin so that we can initialze IoPin(IoPin::null()) explicitly
        //null() cannot be implemented as a function, as it would necessitate a copy, which is prohibited
        class null;

        //set all pins to their (safe) default output:
        static void deactivateAll();
        static void registerExitHandler();

        //prevent copy operations to make pin lifetime-tracking easier.
        //  otherwise, we end up with a pin resetting itself everytime its copied
        IoPin(const IoPin &other) = delete;
        IoPin& operator=(const IoPin &other) = delete;
        //allow the move constructor & move assignment:
        IoPin(IoPin &&other);
        IoPin& operator=(IoPin &&other);

        template <typename ...Args> IoPin(IoPinInversions inversions, Args... args)
          : _pin(args...),  
          _invertReads((inversions & INVERT_READS) != 0), _invertWrites((inversions & INVERT_WRITES) != 0), 
          _defaultState(IO_DEFAULT_NONE) {
            //We need to tell the scheduler to deactivate all pins at shutdown, but only once:
            //Note: this is done in a separate, non-templated function to avoid a bug in gcc-4.7 with the -flto flag
            //Note: only registerExitHandler if this pin is NOT null,
            //  this is critical because the null pin is a static variable, which could possibly get constructed before the Scheduler's exitHandler containers!
            if (!isNull()) {
                registerExitHandler();
                livingPins.insert(this);
            }
        }

        ~IoPin();

        void setDefaultState(DefaultIoState state);

        bool isNull() const;
        IoLevel translateWriteToPrimitive(IoLevel lev) const;
        float translateDutyCycleToPrimitive(float pwm) const;
        const PrimitiveIoPin& primitiveIoPin() const;
        //wrapper functions that take the burden of inversions, etc off the platform-specific drivers:
        void makeDigitalOutput(IoLevel lev);
        void makeDigitalInput();
        IoLevel digitalRead() const;
        void digitalWrite(IoLevel lev);
        void setToDefault();
};

class IoPin::null : public IoPin {
    static null _null;
    public:
        inline null() : IoPin(NO_INVERSIONS, PrimitiveIoPin::null()) {}
        inline static const null& ref() {
            return _null;
        }
};

}
#endif
