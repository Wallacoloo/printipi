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
 * Printipi/motion/exponentialacceleration.h
 *
 * ExponentialAcceleration is an implementation of motion/AccelerationProfile in which 
 * v(t) = f(e^t)
 * This has the possibility to have no instantaneous acceleration, or even impulse.
 * Unfortunately, the current version has v(0) != 0 (so there is instantaneous velocity), and is computationally expensive.
 * For those reasons, I recommend against using this acceleration implementation until it is improved.
 */
 
#ifndef MOTION_EXPONENTIALACCELERATION_H
#define MOTION_EXPONENTIALACCELERATION_H

#include "accelerationprofile.h"
#include <algorithm> //min, isfinite

template <int MaxAccel1000> class ExponentialAcceleration : public AccelerationProfile {
    static constexpr float maxAccel() { return MaxAccel1000 / 1000.; }
    float moveDuration;
    float Vmax;
    public:
        void begin(float moveDuration, float Vmax) {
            this->moveDuration = moveDuration;
            this->Vmax = Vmax;
        }
        float transform(float time) {
            //float Amax = this->driver.maxAccel();
            float Amax = maxAccel();
            float V0 = std::min(0.5*Vmax, 0.1); //c becomes invalid if V0 >= Vmax
            float k = 4*Amax/Vmax;
            float c = V0 / (Vmax-V0);
            if (time > 0.5*moveDuration) {
                return 2*transform(0.5*moveDuration, moveDuration, Vmax) - transform(moveDuration-time, moveDuration, Vmax);
            } else { //take advantage of the fact that comparisons against NaN always compare false to allow for indefinite movements:
                //the problem with the below equation is that it can return infinity if k/Vmax*time is sufficiently large.
                //return 1./k * log(1./c * ((1. + c)*exp(k/Vmax*time) - 1.));
                //aka: 1./k*( log(1./c) + log((1. + c)*exp(k/Vmax*time) - 1.))
                //simplify: 1./k*log(e^x-1) ~=~ 1./k*x at x = Log[1 - E^(-k*.001)], at which point it is only .001 off (however, 1ms is significant! Would rather use a smaller value.
                auto logparam = (1. + c)*exp(k*time) - 1;
                if (std::isfinite(logparam)) {
                    return 1./k*( log(1./c) + log(logparam));
                } else { //use the approximation:
                    return 1./k*(log(1./c) + log(1. + c) + k*time);
                }
            }
        }
};


#endif
