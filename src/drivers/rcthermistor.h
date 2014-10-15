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
 * Printipi/drivers/rcthermistor.h
 *
 * This file provides code to approximate a temperature via first determining the resistance of a thermistor (resistor that varies its resistance according to temperature) via only a digital IO pin, fixed resistor and capacitor.
 * The raspberry pi doesn't have any ADC pins, so we must use the method outlined here: http://www.robotshop.com/media/files/pdf/RCtime_App_Note.pdf
 *
 * NOTE: there are almost certainly better ways to achieve ADC conversion on a Raspberry Pi. 
 *   A discussion on these can be found here: https://github.com/Wallacoloo/printipi/issues/24
 */
 

#ifndef DRIVERS_RCTHERMISTOR_H
#define DRIVERS_RCTHERMISTOR_H

#include <time.h> //for timespec
#include <cmath>
#include "common/typesettings.h" //for EventClockT
#include "common/mathutil.h" //for CtoK, etc
#include "common/logging.h"

namespace drv {

template <typename Pin, unsigned R_OHMS, unsigned C_PICO, unsigned VCC_mV, unsigned V_TOGGLE_mV, unsigned T0_C, unsigned R0_OHMS, unsigned BETA, unsigned MIN_R=0, unsigned MAX_R=R0_OHMS*2> class RCThermistor {
    static constexpr float C = C_PICO * 1.0e-12;
    static constexpr float Vcc = VCC_mV/1000.;
    static constexpr float Va = V_TOGGLE_mV/1000.;
    static constexpr float Ra = R_OHMS;
    static constexpr float T0 = mathutil::CtoK(T0_C); //convert to Kelvin
    static constexpr float R0 = R0_OHMS; //measured resistance of thermistor at T0
    static constexpr float B = BETA; //describes how thermistor changes resistance over the temperature range.
    //struct timespec _startReadTime, _endReadTime;
    Pin pin;
    EventClockT::time_point _startReadTime, _endReadTime;
    public:
        RCThermistor() {
            //initIO();
        }
        void startRead() {
            //bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_INPT);
            pin.makeDigitalInput();
            _startReadTime = EventClockT::now(); //timespecNow();
        }
        bool isReady() {
            //if (bcm2835_gpio_lev(PIN)) { //wait for pin to go LOW.
            if (pin.digitalRead() == IoHigh) { //capacitor is still discharging; not ready.
                return false;
            } else {
                _endReadTime = EventClockT::now(); //timespecNow();
                //prepare IOs for the next read (ie. drain the capacitor that was charged during reading)
                pin.makeDigitalOutput(IoHigh); //output high to drain the capacitor
                //bcm2835_gpio_fsel(PIN, BCM2835_GPIO_FSEL_OUTP);
                //bcm2835_gpio_set(PIN); //output high to drain the capacitor
                return true;
            }
        }
        EventClockT::duration timeSinceStartRead() const {
            return EventClockT::now() - _startReadTime;
        }
        
        float value() const {
            //struct timespec tDelta;
            //tDelta = timespecSub(_endReadTime, _startReadTime);
            //float duration = timespecToFloat(tDelta);
            float duration = std::chrono::duration_cast<std::chrono::duration<float> >(_endReadTime - _startReadTime).count();
            LOGV("time to read resistor: %f\n", duration);
            //now try to guess the resistance:
            float resistance = guessRFromTime(duration);
            LOGV("Resistance guess: %f\n", resistance);
            float temp = temperatureFromR(resistance);
            LOGV("Temperature guess: %f\n", temp);
            return temp;
        }
    private:
        float guessRFromTime(float time) const {
            //equation is: Va = Vcc (1 - Ra/(Ra+Rt)) e^-t/(Rt C)
            //where Va is the minimum voltage sensed as HIGH,
            //  Ra is resistance connecting IO pin to cap,
            //  Rt is resistance of thermistor
            //  C is value of capacitor.
            //equation cannot be solved for Rt symbolically. But it can be solved for t:
            //t = C*Rt*ln(Rt*Vcc/ ((Ra+Rt)*Va));
            //do a binary search for the value of Rt by judging to proximity to t.
            //if the calculated t is < measured t, then Rt is too low. else too high.
            float lower = MIN_R;
            float upper = MAX_R;
            while (upper-lower > 2) {
                float Rt = 0.5*(upper+lower);
                float calcT = C*Rt*log(Rt*Vcc / ((Ra+Rt)*Va));
                if (calcT < time) {
                    lower = Rt;
                } else {
                    upper = Rt;
                }
            }
            return 0.5*(lower+upper);
        }
        float temperatureFromR(float R) const {
            float K = 1. / (1./T0 + log(R/R0)/B); //resistance;
            return mathutil::KtoC(K);
        }
};


}

#endif
