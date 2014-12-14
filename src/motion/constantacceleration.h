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
 * Printipi/motion/constantacceleration.h
 *
 * ConstantAcceleration is an implementation of motion/AccelerationProfile in which 
 *  v(t) = {at [if at < vmax], vmax [if t < duration-accelTime], vmax - a(t-t1) [if t > duration-accelTime] }
 *
 * Polynomial acceleration profiles turn out to be non-trivial, so only constant, linear, and quadratic acceleration have a closed-form solution (above that requires solving the roots of an n+1 degree polynomial. Event just linear acceleration requires solving a degree 3 polynomial.
 */

#ifndef MOTION_CONSTANTACCELERATION_H
#define MOTION_CONSTANTACCELERATION_H

#include "accelerationprofile.h"
#include "common/logging.h"

namespace motion {

class ConstantAcceleration : public AccelerationProfile {
    float _accel;
    float moveDuration;
    float tmax1, tmax2;
    float tbase3;
    float twiceVmax_a;
    inline float a() const { return _accel; }
    public:
        inline ConstantAcceleration(float accel) : _accel(accel) {}
        inline void begin(float moveDuration, float Vmax) {
            this->moveDuration = moveDuration;
            this->tmax1 = Vmax/2/a();
            this->tmax2 = std::isnan(moveDuration) ? INFINITY : moveDuration - Vmax/2/a();
            this->tmax1 = std::min(tmax1, tmax2); //for really short movements, we may not be able to fully accelerate.
            this->tbase3 = moveDuration + Vmax/a(); //TODO: is this the true tbase3 for short movements?
            this->twiceVmax_a = 2*Vmax/a();
            LOGD("Accel::begin dur, Vmax: %f, %f\n", moveDuration, Vmax);
            LOGD("Accel::begin tmax1, tmax2, tbase3, twiceVmax_a: %f, %f, %f, %f\n", tmax1, tmax2, tbase3, twiceVmax_a);
        }
        //float transform(float time, float moveDuration, float Vmax) {
        inline float transform(float time) {
            /*if (time < Vmax/2/a()) { //accelerating
                return std::sqrt(2*Vmax/a()*time);
            } else if (time < moveDuration - Vmax/2/a() || std::isnan(moveDuration)) { //constant velocity
                return time + Vmax/2/a();
            } else { //decelerating. Should never be reached if moveDuration is NAN (ie in homing routine)
                return moveDuration + Vmax/a() - std::sqrt(2*Vmax/a()*(moveDuration-time));
            }*/
            LOGV("Accel::transform: %f\n", time);
            if (time < tmax1) { //accelerating
                return std::sqrt(twiceVmax_a*time);
            } else if (time < tmax2) { //constant velocity
                return time + tmax1; //convenient reuse of semi-constant.
            } else { //decelerating. Should never be reached if moveDuration was NAN (ie in homing routine)
                return tbase3 - std::sqrt(twiceVmax_a*(moveDuration-time));
            }
        }
};

}

#endif
