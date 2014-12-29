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
 * The A4988 is a current-chopping stepper motor driver IC.
 * It is used in the StepStick, Pololu stepper motor drivers, etc.
 * It consists of 2 control pins: STEP and DIRECTION.
 * Documentation: http://www.pololu.com/file/download/a4988_DMOS_microstepping_driver_with_translator.pdf?file_id=0J450
 * Minimum STEP high pulse: 1uS
 * Minimum STEP low pulse:  1uS
 * Low -> High transition on STEP pin trigger the step.
*/


#ifndef DRIVERS_A4988_H
#define DRIVERS_A4988_H

#include <array>
#include <chrono>
#include <utility> //for std::move

#include "iodriver.h"
#include "iopin.h"
#include "outputevent.h"
#include "platforms/auto/chronoclock.h"
#include "motion/axisstepper.h" //for StepDirection

namespace iodrv {

class A4988 : public IODriver {
    IoPin enablePin;
    IoPin stepPin;
    IoPin dirPin;
    public:
        inline A4988(IoPin &&stepPin, IoPin &&dirPin, IoPin &&enablePin) : IODriver(), 
          enablePin(std::move(enablePin)), stepPin(std::move(stepPin)), dirPin(std::move(dirPin)) {
            //default to disabled
            this->enablePin.setDefaultState(IO_DEFAULT_LOW);
            //we want to avoid the step/dir pins from being in a floating state.
            this->stepPin.setDefaultState(IO_DEFAULT_LOW);
            this->dirPin.setDefaultState(IO_DEFAULT_LOW);

            this->stepPin.makeDigitalOutput(IoLow);
            this->dirPin.makeDigitalOutput(IoLow);
            this->enablePin.makeDigitalOutput(IoHigh); //set as output and enable.
        }
        inline void lockAxis() {
            //restrict stepper motors from moving
            enablePin.digitalWrite(IoHigh);
        }
        inline void unlockAxis() {
            //let stepper motors move freely
            enablePin.digitalWrite(IoLow);
        }
        inline std::array<OutputEvent, 3> getEventOutputSequence(EventClockT::time_point evtTime, motion::StepDirection dir) const {
            //A4988 is directed by putting a direction on the DIRPIN, and then
            //sending a pulse on the STEPPIN.
            //It's the low->high transition that triggers the step. 
            //NOTE: documentation says STEP must be LOW for at least 1 uS and then HIGH for at least 1 uS.
            return {{OutputEvent(evtTime, dirPin, dir == motion::StepForward ? IoHigh : IoLow),
                OutputEvent(evtTime, stepPin, IoLow),
                OutputEvent(evtTime+std::chrono::microseconds(8), stepPin, IoHigh)}};
        }
};


}

#endif
