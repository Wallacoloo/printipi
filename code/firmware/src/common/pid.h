#ifndef COMMON_PID_H
#define COMMON_PID_H

/* 
 * Printipi/common/pid.h
 * (c) 2014 Colin Wallace
 *
 * http://en.wikipedia.org/wiki/PID_controller
 * PID provides a Proportional-Integral-Derivative controller that can be used as a control feedback mechanism.
 * Notably, it is used to determine PWM settings for the hotend based on feedback from a thermistor.
 */

#include "common/typesettings.h" //for EventClockT

template <int P1000000, int I1000000=0, int D1000000=0, int ITermMax1000000=2000000, int ITermMin1000000=-ITermMax1000000> class PID {
    static constexpr float P = P1000000 / 1000000.;
    static constexpr float I = I1000000 / 1000000.;
    static constexpr float D = D1000000 / 1000000.;
    static constexpr float IMax() { return ITermMax1000000 / 1000000. / (I1000000 / 1000000.); }
    static constexpr float IMin() { return ITermMin1000000 / 1000000. / (I1000000 / 1000000.); }
    float errorI;
    float lastError;
    //struct timespec lastTime;
    EventClockT::time_point lastTime;
    public:
        PID() : errorI(0), lastError(0), lastTime() {}
        /* notify PID controller of a newly-read error value.
        Returns a recalculated output */
        float feed(float error) {
            float deltaT = refreshTime();
            errorI = std::max(IMin(), std::min(IMax(), errorI+error*deltaT)); //clamp the integral error term.
            float errorD = (error-lastError)/deltaT;
            return P*error + I*errorI + D*errorD;
        }
    private:
        float refreshTime() {
            //struct timespec newTime = timespecNow();
            EventClockT::time_point newTime = EventClockT::now();
            if (lastTime == EventClockT::time_point()) { //no previous time.
                lastTime = newTime;
            }
            //float r = timespecToFloat(timespecSub(newTime, lastTime));
            float r = std::chrono::duration_cast<std::chrono::duration<float> >(newTime-lastTime).count();
            lastTime = newTime;
            return r;
        }
            
};


#endif
