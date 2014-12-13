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

#include <set>
#include <utility> //for std::move
#include "drivers/auto/primitiveiopin.h"
#include "schedulerbase.h" //for SchedulerBase::registerExitHandler


namespace drv {

enum IoPinInversions {
    //bitfield that can be used to indicate whether a pin has logically-inverted reads/writes
    NO_INVERSIONS = 0,
    INVERT_READS  = 1,
    INVERT_WRITES = 2
};

class IoPin {
    PrimitiveIoPin _pin;
    bool _invertReads;
    bool _invertWrites;
    IoLevel _defaultState;
    static std::set<IoPin*> livingPins;

    public:
        //forward-declare a 'null' class for IoPin so that we can initialze IoPin(IoPin::null()) explicitly
        //null() cannot be implemented as a function, as it would necessitate a copy, which is prohibited
        struct null;

        //set all pins to their (safe) default output:
        inline static void deactivateAll() {
            for (auto p : livingPins) {
                p->setToDefault();
            }
        }
        inline static void registerExitHandler() {
            //install exit handler to leave pins in a safe state post-execution.
            static bool doOnce(SchedulerBase::registerExitHandler((void(*)())&deactivateAll, SCHED_IO_EXIT_LEVEL));
            (void)doOnce; //destroy 'unused' warning
        }

        //prevent copy operations to make pin lifetime-tracking easier.
        //  otherwise, we end up with a pin resetting itself everytime its copied
        IoPin(const IoPin &other) = delete;
        IoPin& operator=(const IoPin &other) = delete;
        //allow the move constructor:
        IoPin(IoPin &&other);
        IoPin& operator=(IoPin &&other);

        template <typename ...Args> IoPin(IoPinInversions inversions, IoLevel defaultState, Args... args)
          : _pin(args...),  _invertReads((inversions & INVERT_READS) != 0), _invertWrites((inversions & INVERT_WRITES) != 0), _defaultState(defaultState) {
            //We need to tell the scheduler to deactivate all pins at shutdown, but only once:
            //Note: this is done in a separate, non-templated function to avoid a bug in gcc-4.7 with the -flto flag
            registerExitHandler();
            livingPins.insert(this);
        }
        inline ~IoPin() {
            setToDefault();
            livingPins.erase(this);
        }
        inline bool isNull() const { return _pin.isNull(); }
        inline GpioPinIdType id() const { return _pin.id(); } //TODO: remove this
        inline bool areWritesInverted() const { return _invertWrites; } //TODO: remove this
        //wrapper functions that take the burden of inversions, etc off the platform-specific drivers:
        inline void makeDigitalOutput(IoLevel lev) {
            //set the pin as a digital output, and give it the specified state.
            //Doing these two actions together allow us to prevent the pin from ever being in an undefined state.
            _pin.makeDigitalOutput(_invertWrites ? !lev : lev);
        }
        inline void makeDigitalInput() {
            _pin.makeDigitalInput();
        }
        inline IoLevel digitalRead() const {
            //relay the call to the real pin, performing any inversions necessary
            return _invertReads ? !_pin.digitalRead() : _pin.digitalRead();
        }
        inline void digitalWrite(IoLevel lev) {
            //relay the call to the real pin, performing any inversions necessary
            _pin.digitalWrite(_invertWrites ? !lev : lev);
        }
        inline void setToDefault() {
            //set the pin to a "safe" default state:
            if (!_pin.isNull()) {
                digitalWrite(_defaultState);
            }
        }
};

class IoPin::null : public IoPin {
    static null _null;
    public:
        inline null() : IoPin(NO_INVERSIONS, IoLow, PrimitiveIoPin::null()) {}
        inline static const null& ref() {
            return _null;
        }
};

}
#endif
