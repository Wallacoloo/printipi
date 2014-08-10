#ifndef MOTION_CONSTANTACCELERATION_H
#define MOTION_CONSTANTACCELERATION_H



#include "accelerationprofile.h"
#include "common/logging.h"

template <int Accel1000> class ConstantAcceleration : public AccelerationProfile {
	static constexpr float a() { return Accel1000 / 1000.; }
	float moveDuration;
	float tmax1, tmax2;
	float tbase3;
	float twiceVmax_a;
	public:
		void begin(float moveDuration, float Vmax) {
			moveDuration = moveDuration;
			tmax1 = Vmax/2/a();
			tmax2 = std::isnan(moveDuration) ? INFINITY : moveDuration - Vmax/2/a();
			tbase3 = moveDuration + Vmax/a();
			twiceVmax_a = 2*Vmax/a();
			LOG("Accel::begin dur, Vmax: %f, %f\n", moveDuration, Vmax);
			LOG("Accel::begin tmax1, tmax2, tbase3, twiceVmax_a: %f, %f, %f, %f\n", tmax1, tmax2, tbase3, twiceVmax_a);
		}
		//float transform(float time, float moveDuration, float Vmax) {
		float transform(float time) {
			/*if (time < Vmax/2/a()) { //accelerating
				return std::sqrt(2*Vmax/a()*time);
			} else if (time < moveDuration - Vmax/2/a() || std::isnan(moveDuration)) { //constant velocity
				return time + Vmax/2/a();
			} else { //decelerating. Should never be reached if moveDuration is NAN (ie in homing routine)
				return moveDuration + Vmax/a() - std::sqrt(2*Vmax/a()*(moveDuration-time));
			}*/
			LOGV("Accel::transform: %f\n", time);
			if (time < tmax1) { //accelerating
				return std::sqrt(twiceVmax_a*time);
			} else if (time < tmax2) { //constant velocity
				return time + tmax1; //convenient reuse of semi-constant.
			} else { //decelerating. Should never be reached if moveDuration was NAN (ie in homing routine)
				return tbase3 - std::sqrt(twiceVmax_a*(moveDuration-time));
			}
		}
};

#endif
