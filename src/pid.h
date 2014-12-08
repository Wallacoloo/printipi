	#ifndef PID_H
#define PID_H

/* 
 * Printipi/pid.h
 *
 * http://en.wikipedia.org/wiki/PID_controller
 * PID provides a Proportional-Integral-Derivative controller that can be used as a control feedback mechanism.
 * Notably, it is used to determine PWM settings for the hotend based on feedback from a thermistor.
 */

#include <chrono> //for std::chrono::duration
#include "drivers/auto/chronoclock.h" //for EventClockT

class PID {
    float _P, _I, _D;
    float errorI;
    float lastValue;

    EventClockT::time_point lastTime;
    float P() const { return _P; }
    float I() const { return _I; }
    float D() const { return _D; }
    public:
        PID(float P, float I, float D) 
          : _P(P), _I(I), _D(D),
            errorI(0), lastValue(0), lastTime() {}
        /* notify PID controller of a newly-read error value.
        Returns a recalculated output */
        inline float feed(float setpoint, float pv) {
       	    float error = setpoint - pv;
            float deltaT = refreshTime();
            // Use a simple 1st order finite difference for the derivative - no fancy filtering.
            float errorD = (pv-lastValue)/deltaT;
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
    private:
        inline float refreshTime() {
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
