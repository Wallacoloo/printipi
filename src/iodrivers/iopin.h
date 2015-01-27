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

#ifndef IODRIVERS_IOPIN_H
#define IODRIVERS_IOPIN_H

#include <set>
#include <utility> //for std::move
#include "platforms/auto/primitiveiopin.h"

namespace iodrv {

enum IoPinInversions {
    //bitfield that can be used to indicate whether a pin has logically-inverted reads/writes
    NO_INVERSIONS = 0,
    INVERT_READS  = 1,
    INVERT_WRITES = 2,
};
//bitwise OR operator for IoPinInversions, to avoid INVERT_READS|INVERT_WRITES resulting in an integer type instead of a IoPinInversions type.
inline IoPinInversions operator|(IoPinInversions a, IoPinInversions b) {
    return static_cast<IoPinInversions>(static_cast<int>(a) | static_cast<int>(b));
}

enum DefaultIoState {
    IO_DEFAULT_NONE,
    IO_DEFAULT_HIGH_IMPEDANCE,
    IO_DEFAULT_LOW,
    IO_DEFAULT_HIGH
};

//Used only in debug mode to detect errors when a write is attempted on a pin set to input mode, etc.
enum IoPinMode {
    IOPIN_MODE_UNSPECIFIED,
    IOPIN_MODE_INPUT,
    IOPIN_MODE_OUTPUT,
    IOPIN_MODE_PWM,
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
    //if compiling in debug mode, we should force that all users of IoPin correctly set it to output mode before writing to it.
    #ifndef NDEBUG
        IoPinMode _currentMode;
    #endif

    static std::set<IoPin*> livingPins;
    //called internally to register an exit handler that safely deactivates all IoPins upon program termination.
    static void registerExitHandler();
    public:
        //forward-declare a 'null' class for IoPin so that we can initialze IoPin(IoPin::null()) explicitly
        //null() cannot be implemented as a function, as it would necessitate a copy, which is prohibited
        class null;

        //set all pins to their (safe) default output:
        static void deactivateAll();

        //prevent copy operations to make pin lifetime-tracking easier.
        //  otherwise, we end up with a pin resetting itself everytime its copied
        IoPin(const IoPin &other) = delete;
        IoPin& operator=(const IoPin &other) = delete;
        //allow the move constructor
        IoPin(IoPin &&other);
        //allow move assignment
        IoPin& operator=(IoPin &&other);

        //Create an IoPin given its inversions, and forward the remaining arguments to PrimitiveIoPin
        template <typename ...Args> IoPin(IoPinInversions inversions, Args... args)
          : _pin(args...),  
          _invertReads((inversions & INVERT_READS) != 0), _invertWrites((inversions & INVERT_WRITES) != 0), 
          _defaultState(IO_DEFAULT_NONE) {
            #ifndef NDEBUG
                _currentMode = IOPIN_MODE_UNSPECIFIED;
            #endif
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
        //@return true if the underlying PrimitiveIoPin is null (either IoPin was construction via IoPin::null(), or IoPin(..., PrimitiveIoPin::null()))
        bool isNull() const;
        //Convert a write level such that IoPin.digitalWrite(lev) is the same as IoPin.primitiveIoPin().digitalWrite(<translated level>)
        //  where <translated level> is the value returned by this function
        IoLevel translateWriteToPrimitive(IoLevel lev) const;
        float translateDutyCycleToPrimitive(float pwm) const;
        const PrimitiveIoPin& primitiveIoPin() const;
        //set the pin as a digital output, and give it the specified state.
        //Doing these two actions together allow us to prevent the pin from ever being in an undefined state.
        void makeDigitalOutput(IoLevel lev);
        //set the pin as a pwm output & give it the desired duty / period.
        //Doing these two actions together allow us to prevent the pin from ever being in an undefined state.
        void makePwmOutput(float duty, EventClockT::duration desiredPeriod=EventClockT::duration(0));
        //Configure the pin as an input
        void makeDigitalInput();
        //Read a binary logic level from the pin. MUST first call makeDigitalInput() to put the pin in input mode.
        IoLevel digitalRead() const;
        //Write a binary logic level to the pin (IoHigh or IoLow). MUST first call makeDigitalOutput() to put the pin in output mode.
        void digitalWrite(IoLevel lev);
        //Set the pin to output a PWM signal. MUST first call makePwmOutput() to put the pin in pwm mode.
        //@duty proportion of time that the pin should be ACTIVE (0.0 - 1.0).
        //@desiredPeriod *desired* PWM cycle length (the actual length isn't guaranteed). Useful for decreasing fet/relay switching frequency, etc. 
        void pwmWrite(float duty, EventClockT::duration desiredPeriod=EventClockT::duration(0));
        //put the pin into its default state, as set by setDefaultState(...).
        void setToDefault();
};

class IoPin::null : public IoPin {
    public:
        inline null() : IoPin(NO_INVERSIONS, PrimitiveIoPin::null()) {}
};

}
#endif
