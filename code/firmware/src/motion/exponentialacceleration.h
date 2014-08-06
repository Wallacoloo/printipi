#ifndef MOTION_EXPONENTIALACCELERATION_H
#define MOTION_EXPONENTIALACCELERATION_H

#include "accelerationprofile.h"
#include <algorithm> //min, isfinite

template <int MaxAccel1000> class ExponentialAcceleration : public AccelerationProfile {
	static constexpr float maxAccel() { return MaxAccel1000 / 1000.; }
	public:
		float transform(float time, float moveDuration, float Vmax) {
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
