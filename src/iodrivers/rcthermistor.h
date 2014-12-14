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
 * Printipi/iodrivers/rcthermistor.h
 *
 * This file provides code to approximate a temperature via first determining the resistance of a thermistor (resistor that varies its resistance according to temperature) via only a digital IO pin, fixed resistor and capacitor.
 * The raspberry pi doesn't have any ADC pins, so we must use the method outlined here (figure 1): 
 *   http://www.robotshop.com/media/files/pdf/RCtime_App_Note.pdf
 *
 * NOTE: there are almost certainly better ways to achieve ADC conversion on a Raspberry Pi. 
 *   A discussion on these can be found here: https://github.com/Wallacoloo/printipi/issues/24
 */
 

#ifndef DRIVERS_RCTHERMISTOR_H
#define DRIVERS_RCTHERMISTOR_H

#include <cmath>
#include "platforms/auto/chronoclock.h" //for EventClockT
#include "common/mathutil.h" //for CtoK, etc
#include "common/logging.h"

namespace iodrv {

template <typename Pin> class RCThermistor {
    //Note: R_OHMS should be at least 300 ohms to limit current through the pins, 
    //  but you probably don't want it higher than 1000 ohms, or else you won't be able to sense high temperatures.
    //Larger capacitors give you more precision, but decrease the frequency with which you can measure when at a low temperature (e.g. < 50 C).
    //V_TOGGLE_V is the voltage threshold at which your pin will switch from sensing HIGH to LOW
    //  note: due to hysterisis, this may not be the same voltage at which it switches from LOW to HIGH.
    //T0, R0 and BETA are constants for the thermistor and should be found on the packaging / documentation.
    // (T0 can be assumed to be 25*C if not explicitly listed)
    float C, Vcc, Va, Ra; //capacitance, supply voltage, toggle voltage and fixed RC resistance, respectively
    float T0, R0; //R0 = measured resistance at temperature T0 (in Ohms / Kelvin) (listed on thermistor packaging or documentation page)
    float B; //Thermistor Beta value; describes how thermistor changes resistance over the temperature range (listed on thermistor packaging or documentation page)
    float MIN_R, MAX_R; //thermistor resistance range to consider when estimating the resistance.
    Pin pin;
    EventClockT::time_point _startReadTime, _endReadTime;
    public:
        inline RCThermistor(float R_OHMS, float C_FARADS, float VCC_V, float V_TOGGLE_V, float T0_C, float R0_OHMS, float BETA)
          : C(C_FARADS), Vcc(VCC_V), Va(V_TOGGLE_V), Ra(R_OHMS), T0(mathutil::CtoK(T0_C)), R0(R0_OHMS), B(BETA),
          MIN_R(0), MAX_R(R0_OHMS*2) {}
        inline void startRead() {
            pin.makeDigitalInput();
            _startReadTime = EventClockT::now();
        }
        inline bool isReady() {
            if (pin.digitalRead() == IoHigh) { //capacitor is still discharging; not ready.
                return false;
            } else {
                //reading is complete. Log the current time to determine discharge duration:
                _endReadTime = EventClockT::now();
                //prepare IOs for the next read (ie. drain the capacitor that was charged during reading)
                pin.makeDigitalOutput(IoHigh); //output high to drain the capacitor
                return true;
            }
        }
        inline EventClockT::duration timeSinceStartRead() const {
            //need to expose this information to assist in detecting freezes / failed reads.
            return EventClockT::now() - _startReadTime;
        }
        
        inline float value() const {
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
        inline float guessRFromTime(float time) const {
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
        inline float temperatureFromR(float R) const {
            float K = 1. / (1./T0 + log(R/R0)/B); //resistance;
            return mathutil::KtoC(K);
        }
};


}

#endif
