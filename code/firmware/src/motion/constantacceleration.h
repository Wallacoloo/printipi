#ifndef MOTION_CONSTANTACCELERATION_H
#define MOTION_CONSTANTACCELERATION_H



#include "accelerationprofile.h"

template <int Accel1000> class ConstantAcceleration : public AccelerationProfile {
	static constexpr float a() { return Accel1000 / 1000.; }
	public:
		//void begin
		float transform(float time, float moveDuration, float Vmax) {
			if (time < Vmax/2/a()) { //accelerating
				return std::sqrt(2*Vmax/a()*time);
			} else if (time < moveDuration - Vmax/2/a() || std::isnan(moveDuration)) { //constant velocity
				return time + Vmax/2/a();
			} else { //decelerating. Should never be reached if moveDuration is NAN (ie in homing routine)
				return moveDuration + Vmax/a() - std::sqrt(2*Vmax/a()*(moveDuration-time));
			}
		}
};

#endif
