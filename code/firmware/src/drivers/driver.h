#ifndef DRIVERS_DRIVER_H
#define DRIVERS_DRIVER_H

#include <cstddef> //for size_t
#include "../event.h" //for gparse::StepDirection

namespace drv {

class Driver {
	public:
		//M105
		virtual void getTemperature(int &extruder, int& platform) const;
		//virtual std::size_t numAxis() const = 0;
		//constexpr std::size_t numAxis() const;
		virtual float defaultMoveRate() const = 0;
		virtual float defaultFeedRate() const = 0;
		virtual float relativeTimeOfNextStep(int axisIdx, StepDirection &dir, float x, float y, float z, float e, float velx, float vely, float velz, float velExt) const = 0;
};

}
#endif
