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


#ifndef IODRIVERS_STEPDIRSTEPPERDRIVER_H
#define IODRIVERS_STEPDIRSTEPPERDRIVER_H

#include <array>
#include <chrono>
#include <utility> //for std::move

#include "iodriver.h"
#include "iopin.h"
#include "outputevent.h"
#include "platforms/auto/chronoclock.h"
//for StepDirection
#include "motion/axisstepper.h" 

namespace iodrv {

/*
 * A "StepDirStepperDriver" is used to generically drive any stepper motor driver chip that accepts a "step" input and a "direction" input.
 *   This includes A4988 and DRV8825 current-chopping stepper driver ICs
 *
 * Documentation for A4988 stepper driver: http://www.pololu.com/file/download/a4988_DMOS_microstepping_driver_with_translator.pdf?file_id=0J450
 * Documentation for DRV8825: https://www.pololu.com/product/2133
 * Low -> High transition on STEP pin trigger the step.
 * Minimum STEP high pulse: 1uS (A4988), 1.9uS (DRV8825)
 * Minimum STEP low pulse:  1uS (A4988), 1.9uS (DRV8825)
*/
class StepDirStepperDriver : public IODriver {
    IoPin enablePin;
    IoPin stepPin;
    IoPin dirPin;
    public:
        inline StepDirStepperDriver(IoPin &&stepPin, IoPin &&dirPin, IoPin &&enablePin) : IODriver(), 
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
        //@inherit
        inline void lockAxis() {
            //restrict stepper motors from moving
            enablePin.digitalWrite(IoHigh);
        }
        //@inherit
        inline void unlockAxis() {
            //let stepper motors move freely
            enablePin.digitalWrite(IoLow);
        }
        inline std::array<OutputEvent, 3> getEventOutputSequence(EventClockT::time_point evtTime, motion::StepDirection dir) const {
            //A4988 is directed by putting a direction on the DIRPIN, and then
            //sending a pulse on the STEPPIN.
            //It's the low->high transition that triggers the step. 
            //NOTE: documentation says STEP must be LOW for at least 1 uS and then HIGH for at least 1 uS (for A4988)
            //DRV8825 requires 1.9 uS here.
            return {{OutputEvent(evtTime, dirPin, dir == motion::StepForward ? IoHigh : IoLow),
                OutputEvent(evtTime, stepPin, IoLow),
                OutputEvent(evtTime+std::chrono::microseconds(8), stepPin, IoHigh)}};
        }
};


}

#endif
