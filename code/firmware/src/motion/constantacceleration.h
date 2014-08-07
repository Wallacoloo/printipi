#ifndef MOTION_CONSTANTACCELERATION_H
#define MOTION_CONSTANTACCELERATION_H

/* Canceled.
  Implementation is non-trivial; requires solving the roots of a polynomial of degree N+1.
  Therefore, only constant-acceleration is trivial.
*/

#include "accelerationprofile.h"

template <int Accel1000> class ConstantAcceleration : public AccelerationProfile {
	static constexpr float a() { return Accel1000 / 1000.; }
	public:
		float transform(float time, float moveDuration, float Vmax) {
			if (time < Vmax/2/a()) { //accelerating
				return std::sqrt(2*Vmax/a()*time);
			} else if (time < moveDuration - Vmax/2/a()) { //constant velocity
				return time + Vmax/2/a();
			} else { //decelerating
				return moveDuration + Vmax/a() - std::sqrt(2*Vmax/a()*(moveDuration-time));
			}
		}
};

#endif
