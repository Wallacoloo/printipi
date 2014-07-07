#ifndef DRIVERS_AXISSTEPPER_H
#define DRIVERS_AXISSTEPPER_H

#include "../event.h"

namespace drv {

class AxisStepper {
	public:
		float time;
		StepDirection direction;
};

}

#endif
