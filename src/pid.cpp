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

#include "pid.h"

#include <chrono> //for std::chrono::duration

float PID::feed(float setpoint, float pv) {
    float error = setpoint - pv;
    float deltaT = refreshTime();
    // Use a simple 1st order finite difference for the derivative - no fancy filtering.
    // Note the negation: this is because the D term is the derivative of the *error* (setpoint - pv).
    float errorD = -(pv-lastValue)/deltaT;
    lastValue = pv;
    // Then figure out the change for the integral
    float update = I() * error * deltaT;
    errorI += update;
    // Then compute the output, saturate it to [0,1], and implement anti-windup
    float output = P()*error + errorI + D()*errorD;

    if(output < 0.0){
        if(error < 0.0){
            errorI -= update;
        }
        return 0.0;
    }

    if(1.0 < output){
        if(error > 0.0){
            errorI -= update;
        }
        return 1.0;
    }

    return output;
}

float PID::refreshTime() {
    EventClockT::time_point newTime = EventClockT::now();
    if (lastTime == EventClockT::time_point()) { //no previous time.
        lastTime = newTime;
    }
    float r = std::chrono::duration_cast<std::chrono::duration<float> >(newTime-lastTime).count();
    lastTime = newTime;
    return r;
}