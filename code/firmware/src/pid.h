#ifndef PID_H
#define PID_H

/* 
 * Printipi/pid.h
 * (c) 2014 Colin Wallace
 *
 * http://en.wikipedia.org/wiki/PID_controller
 * PID provides a Proportional-Integral-Derivative controller that can be used as a control feedback mechanism.
 * Notably, it is used to determine PWM settings for the hotend based on feedback from a thermistor.
 */

#include "common/timeutil.h" //for timespec*

template <int P1000000, int I1000000=0, int D1000000=0> class PID {
	static constexpr float P = P1000000 / 1000000.;
	static constexpr float I = I1000000 / 1000000.;
	static constexpr float D = D1000000 / 1000000.;
	float errorI;
	float lastError;
	struct timespec lastTime;
	public:
		PID() : errorI(0), lastError(0), lastTime{0, 0} {}
		/* notify PID controller of a newly-read error value.
		Returns a recalculated output */
		float feed(float error) {
			float deltaT = refreshTime();
			errorI += error*deltaT;
			float errorD = (error-lastError)/deltaT;
			return P*error + I*errorI + D*errorD;
		}
	private:
		float refreshTime() {
			struct timespec newTime = timespecNow();
			if (lastTime.tv_sec == 0) {
				lastTime = newTime;
			}
			float r = timespecToFloat(timespecSub(newTime, lastTime));
			lastTime = newTime;
			return r;
		}
			
};


#endif
