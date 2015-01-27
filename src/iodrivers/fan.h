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


#ifndef IODRIVERS_FAN_H
#define IODRIVERS_FAN_H
 
#include <utility> //for std::move
#include "iodriver.h"
#include "iopin.h"

namespace iodrv {

/* 
 * The Fan class serves to control a physical Fan, often used to cool plastic as it exits the hotend.
 * This class essentially wraps an IoPin so that it can be commanded and recognized as a fan.
 */
class Fan : public IODriver {
    IoPin pin;
    float multiplier;
    EventClockT::duration period;
    public:
        //@pin the IoPin that powers the fan.
        //  In most cases, the fan should NOT be directly powered from the microcontroller. 
        //  It should either be behind a transistor whose input is connected to @pin,
        //  or (if its a 3 wire fan), the fan should be powered from the power rail & the control signal (3rd wire) should be connected to @pin
        //@defaultState the state to place this pin in upon shutdown
        //@multiplier is a weight to apply to the desired PWM value.
        //  This is useful to allow M106 to set this fan to a different value than the rest.
        //@period the desired cycle length (in seconds) to use for PWM control. Most users will be happy keeping this at the default, 0.
        //  This is useful if using a transistor that cannot switch fast (like a relay).
        inline Fan(IoPin &&pin, DefaultIoState defaultState=IO_DEFAULT_NONE, float multiplier=1.0, EventClockT::duration period=EventClockT::duration())
          : pin(std::move(pin)), multiplier(multiplier), period(period) {
            this->pin.setDefaultState(defaultState);
            this->pin.makePwmOutput(0.0);
        }
        inline bool isFan() const { return true; }
        //called by State upon receiving an M106 gcode
        inline void setFanDutyCycle(float dutyCycle) {
            (void)dutyCycle;
            pin.pwmWrite(dutyCycle*multiplier, period);
        }
};

}
#endif
