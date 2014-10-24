#ifndef COMMON_PID_H
#define COMMON_PID_H

/* 
 * Printipi/common/pid.h
 *
 * http://en.wikipedia.org/wiki/PID_controller
 * PID provides a Proportional-Integral-Derivative controller that can be used as a control feedback mechanism.
 * Notably, it is used to determine PWM settings for the hotend based on feedback from a thermistor.
 */

#include <chrono> //for std::chrono::duration
#include "drivers/auto/chronoclock.h" //for EventClockT

template <int P1000000, int I1000000=0, int D1000000=0, int ITermMax1000000=2000000, int ITermMin1000000=-ITermMax1000000> class PID {
    static constexpr float P = P1000000 / 1000000.;
    static constexpr float I = I1000000 / 1000000.;
    static constexpr float D = D1000000 / 1000000.;
    //static constexpr float IMax() { return ITermMax1000000 / 1000000. / (I1000000 / 1000000.); }
    //static constexpr float IMin() { return ITermMin1000000 / 1000000. / (I1000000 / 1000000.); }
    float errorI;
    float lastValue;

    // Switch to a higher-order discrete derivative operator (3rd?)
    // Differentiate the process value, not the set-point
    // Anti-windup mechanism
    // The eternal question - scale then sum, or sum then scale

    EventClockT::time_point lastTime;
    public:
        PID() : errorI(0), lastValue(0), lastTime() {}
        /* notify PID controller of a newly-read error value.
        Returns a recalculated output */
        float feed(float setpoint, float pv) {
       	    float error = setpoint - pv;
            float deltaT = refreshTime();
            // Use a simple 1st order finite difference for the derivative - no fancy filtering.
            float errorD = (pv-lastValue)/deltaT;
            lastValue = pv;
            // Then figure out the change for the integral integral
            float update = error * deltaT;
            errorI += update;
            // Then compute the output, saturate it to [0,1], and implement anti-windup
            float output = P*error + I*errorI + D*errorD;

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
    private:
        float refreshTime() {
            EventClockT::time_point newTime = EventClockT::now();
            if (lastTime == EventClockT::time_point()) { //no previous time.
                lastTime = newTime;
            }
            float r = std::chrono::duration_cast<std::chrono::duration<float> >(newTime-lastTime).count();
            lastTime = newTime;
            return r;
        }
            
};


#endif
