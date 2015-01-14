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
 

#ifndef DRIVERS_RCTHERMISTOR_H
#define DRIVERS_RCTHERMISTOR_H

#include <cmath>
#include <utility> //for std::move
#include "platforms/auto/chronoclock.h" //for EventClockT
#include "iodrivers/iopin.h"
#include "common/mathutil.h" //for CtoK, etc
#include "common/logging.h"

namespace iodrv {


/*
 * This class provides code to approximate a temperature via first determining the resistance of a thermistor 
 *   (resistor that varies its resistance according to temperature) via only a digital IO pin, fixed resistor and capacitor.
 * The raspberry pi doesn't have any ADC pins, so we must use the method outlined below.
 * The implementation is designed to be compatible with ramps-fd.
 *
 *                Vcc
 *                 \
 *                 / Rup
 *                 \
 *         Rseries /        Rchrg
 *      o---/\/\/\-+-----o-/\/\/\-o
 *      \         _|_             |
 *      / therm   ___ C           |
 *      \          |              |
 *   THERMPIN     GND          CHRG/MEAS
 *
 *  On ramps-fd Rup = 4.7k, Rseries = 22 ohm.
 *  Rchrg can be anything > ~200 ohms - make sure not to draw more than 16 mA from a pin.
 *  If drawing a significant amount of current, make sure to configure the Pi's pin drive strength first.
 *  Recommended Rchrg value is 1k ohm. The ratio Rchrg / Rup should be significantly less than 0.5 to ensure that the pin state actually changes
 *  Also, Rseries should be such that the thermistor resistance + Rseries is always such that < 16 mA is drawn.
 *
 *  Note: in the case that Rchrg = 0, therm = infinite, discharge time = 0.69*RC = 32.8 mS with Rup=4k7, C=10.1e-6, Vtoggle=1.65
 *    so that's the absolute longest discharge time.
 *
 *  Solve for Vcap(t) during a read:
 *  Iin = (Vcc-Vcap)/Rup + (Vcc-Vcap)/(therm+Rseries) where Iin is the current entering C, and THERMPIN is tied to +Vcc
 *  capacitor equation: Q = CV, or, I = C*dV/dt
 *  So, (Vcc-Vcap) * (1/Rup + 1/(therm+Rseries)) = C*Vcap'
 *    (Vcc-Vcap) * (Rup + therm + Rseries) / (C*Rup*(therm+Rseries)) = Vcap'
 *  Vcc-Vcap = Vcap' * (C*Rup*(therm+Rseries)) / (Rup + therm + Rseries)
 *  Vcap'*(C*Rup*(therm+Rseries))/(Rup + therm + Rseries) + Vcap = Vcc
 *    let Vcap = (V0-Vfinal)*e^(l*t) + Vfinal
 *    Consider the circuit configured with THERMPIN = no-connect, CHRG/MEAS=gnd for a long time before t=0:
 *    then Vcap = k*e^(l*t) + Vcc where k = V0-Vcc.
 *      V0 = Vcc * Rchrg / (Rchrg + Rup),
 *      so *k = Vcc*(Rchrg / (Rchrg + Rup) - 1)
 *    then Vcap' = k*l*e^(l*t)
 *  k*l*e^(l*t)*(C*Rup*(therm+Rseries))/(Rup + therm + Rseries) + k*e^(l*t) + Vcc = Vcc
 *  k*e^(l*t) * [l*(C*Rup*(therm+Rseries))/(Rup + therm + Rseries) + 1] = 0
 *    So, l = -1 / [(C*Rup*(therm+Rseries))/(Rup + therm + Rseries)]
 *    *l = -(Rup + therm + Rseries) / (C*Rup*(therm+Rseries))
 *
 *  From the read, we can know Vcap(tr) = Vtoggle
 *  From this, we want to solve for therm:
 *  Vtoggle = k * e^(l*tr) + Vcc
 *  (Vtoggle-Vcc)/k = e^(l*tr)
 *  1.0/tr*ln((Vtoggle-Vcc)/k) = l = -(Rup + therm + Rseries) / (C*Rup*(therm+Rseries))
 *  C*Rup*(therm+Rseries)/tr*ln((Vtoggle-Vcc)/k) = -(Rup + therm + Rseries)
 *  therm* C*Rup/tr*ln((Vtoggle-Vcc)/k) + C*Rup*Rseries/tr*ln((Vtoggle-Vcc)/k) + therm = -Rup - Rseries
 *  therm[1+C*Rup/tr*ln((Vtoggle-Vcc)/k)] = C*Rup*Rseries/tr*ln((Vtoggle-Vcc)/k) - Rup - Rseries
 *  let denom = [1+C*Rup/tr*ln((Vtoggle-Vcc)/k)],
 *  then therm = (C*Rup*Rseries/tr*ln((Vtoggle-Vcc)/k) - Rup - Rseries) / denom
 */
class RCThermistor2Pin {
    IoPin thermPin;
    IoPin chargeMeasPin;
    float C, Vcc, Rup, Rchrg, Rseries;
    float Vtoggle;
    //R0 = measured resistance at temperature T0 (in Ohms and Kelvin) (listed on thermistor packaging or documentation page)
    float T0, R0; 
    //Thermistor Beta value; describes how thermistor changes resistance over the temperature range (listed on thermistor packaging or documentation page)
    float B;
    EventClockT::time_point _startReadTime, _endReadTime;
    public:
        inline RCThermistor2Pin(IoPin &&thermPin, IoPin &&chargeMeasPin, float RCHRG_OHMS, float RSERIES_OHMS, float RUP_OHMS,
            float C_FARADS, float VCC_V, float V_TOGGLE_V, float T0_C, float R0_OHMS, float BETA)
          : thermPin(std::move(thermPin)), chargeMeasPin(std::move(chargeMeasPin)), 
            C(C_FARADS), Vcc(VCC_V), Rup(RUP_OHMS), Rchrg(RCHRG_OHMS), Rseries(RSERIES_OHMS),
            Vtoggle(V_TOGGLE_V), T0(mathutil::CtoK(T0_C)), R0(R0_OHMS), B(BETA) {
            thermPin.setDefaultState(IO_DEFAULT_HIGH_IMPEDANCE);
            chargeMeasPin.setDefaultState(IO_DEFAULT_HIGH_IMPEDANCE);
        }
        inline void startRead() {
            _startReadTime = EventClockT::now();
            //disconnect charge pin from ground and use it to measure
            chargeMeasPin.makeDigitalInput();
            //tie thermistor to high to begin drain
            thermPin.makeDigitalOutput(IoHigh);
        }
        //called during the read.
        //@return true if the read is complete (ready to call value())
        inline bool isReady() {
            if (chargeMeasPin.digitalRead() == IoLow) { //capacitor is still discharging; not ready.
                return false;
            } else {
                //reading is complete. Log the current time to determine discharge duration:
                _endReadTime = EventClockT::now();
                //disconnect thermistor to allow the capacitor to be drained
                thermPin.makeDigitalInput();
                //prepare IOs for the next read (ie. drain the capacitor that was charged during reading)
                chargeMeasPin.makeDigitalOutput(IoLow);
                return true;
            }
        }
        //need to expose this information to assist in detecting freezes / failed reads.
        inline EventClockT::duration timeSinceStartRead() const {
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
        //@tr the time it took to charge the capacitor through the thermistor
        inline float guessRFromTime(float tr) const {
            //These equations are derived at the top of the file.
            float k = Vcc*(Rchrg / (Rchrg + Rup) - 1);
            float denom = 1+C*Rup/tr*log((Vtoggle-Vcc)/k);
            float resistance = (C*Rup*Rseries/tr*log((Vtoggle-Vcc)/k) - Rup - Rseries) / denom;
            return resistance;
        }
        inline float temperatureFromR(float R) const {
            float K = 1. / (1./T0 + log(R/R0)/B); //resistance;
            return mathutil::KtoC(K);
        }
};


}

#endif
