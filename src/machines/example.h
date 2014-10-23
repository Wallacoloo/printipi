#ifndef DRIVERS_MACHINES_EXAMPLE_H
#define DRIVERS_MACHINES_EXAMPLE_H

#include <tuple>
#include "motion/constantacceleration.h"
#include "drivers/linearcoordmap.h"
#include "machines/machine.h"

namespace machines {

using namespace drv; //for all the drivers

class Example : public Machine {
    public:
        typedef ConstantAcceleration<500*1000> AccelerationProfileT;
        typedef LinearCoordMap<> CoordMapT;
        typedef std::tuple<> AxisStepperTypes;
        typedef std::tuple<> IODriverTypes;
};

}

#endif
