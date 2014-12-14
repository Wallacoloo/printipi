#include "pid.h"

#include <chrono> //for std::chrono::duration

float PID::feed(float setpoint, float pv) {
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

float PID::refreshTime() {
    EventClockT::time_point newTime = EventClockT::now();
    if (lastTime == EventClockT::time_point()) { //no previous time.
        lastTime = newTime;
    }
    float r = std::chrono::duration_cast<std::chrono::duration<float> >(newTime-lastTime).count();
    lastTime = newTime;
    return r;
}