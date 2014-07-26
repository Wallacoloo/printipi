#ifndef PID_H
#define PID_H

#include "timeutil.h" //for timespec*


template <int P1000000, int I1000000=0, int D1000000=0> class PID {
	static constexpr float P = P1000000 / 1000000.;
	static constexpr float I = I1000000 / 1000000.;
	static constexpr float D = D1000000 / 1000000.;
	float errorI;
	float lastError;
	struct timespec lastTime;
	public:
		PID() : lastTime{0, 0} {}
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
