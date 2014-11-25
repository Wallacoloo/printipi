#ifndef DRIVERS_MACHINES_EXAMPLE_H
#define DRIVERS_MACHINES_EXAMPLE_H

#include <tuple>
#include "motion/constantacceleration.h"
#include "drivers/linearcoordmap.h"
#include "machines/machine.h"

namespace machines {
namespace generic {

using namespace drv; //for all the drivers

class Example : public Machine {
    public:
        ConstantAcceleration<500*1000> getAccelerationProfile() const {
            return ConstantAcceleration<500*1000>();
        }
        LinearCoordMap<> getCoordMap() const {
            return LinearCoordMap<>();
        }
        std::tuple<> getAxisSteppers() const {
            return std::tuple<>();
        }
        std::tuple<> getIoDrivers() const {
            return std::tuple<>();
        }
};

}
}

#endif
