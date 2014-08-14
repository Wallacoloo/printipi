#ifndef DRIVERS_MACHINES_EXAMPLE_H
#define DRIVERS_MACHINES_EXAMPLE_H

#include <tuple>
#include "motion/constantacceleration.h"
#include "drivers/linearcoordmap.h"
#include "drivers/machines/machine.h"

namespace drv {

class Example : public Machine {
	public:
		typedef ConstantAcceleration<500*1000> AccelerationProfileT;
		typedef LinearCoordMap<> CoordMapT;
		typedef std::tuple<> AxisStepperTypes;
		typedef std::tuple<> IODriverTypes;
};


}

#endif
