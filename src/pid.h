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
