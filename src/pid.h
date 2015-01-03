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

#ifndef PID_H
#define PID_H

//for EventClockT
#include "platforms/auto/chronoclock.h"


/* 
 * PID provides a Proportional-Integral-Derivative controller that can be used as a control feedback mechanism.
 * Notably, it is used to determine PWM settings for the hotend based on feedback from a thermistor.
 * PID workings: http://en.wikipedia.org/wiki/PID_controller
 */
class PID {
    float _P, _I, _D;
    float errorI;
    float lastValue;

    EventClockT::time_point lastTime;
    inline float P() const { return _P; }
    inline float I() const { return _I; }
    inline float D() const { return _D; }
    public:
        inline PID(float P, float I, float D) 
          : _P(P), _I(I), _D(D),
            errorI(0), lastValue(0), lastTime() {}
        /* notify PID controller of a newly-read error value.
        Returns a recalculated output */
        float feed(float setpoint, float pv);
    private:
        float refreshTime();
            
};


#endif
