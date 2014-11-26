#ifndef DRIVERS_MACHINES_EXAMPLE_H
#define DRIVERS_MACHINES_EXAMPLE_H

#include <tuple>
#include "motion/constantacceleration.h"
#include "drivers/linearcoordmap.h"
#include "machines/machine.h"
#include "common/matrix.h"

namespace machines {
namespace generic {

using namespace drv; //for all the drivers

class Example : public Machine {
    public:
        ConstantAcceleration<500*1000> getAccelerationProfile() const {
            return ConstantAcceleration<500*1000>();
        }
        LinearCoordMap<> getCoordMap() const {
            return LinearCoordMap<>(1.0, 1.0, 1.0, 1.0, Matrix3x3(
            1, 0, 0,
            0, 1, 0,
            0, 0, 1));
        }
        std::tuple<> getAxisSteppers() const {
            return std::tuple<>();
        }
        std::tuple<> getHomeSteppers() const {
            return std::tuple<>();
        }
        std::tuple<> getArcSteppers() const {
            return std::tuple<>();
        }
        std::tuple<> getIoDrivers() const {
            return std::tuple<>();
        }
};

}
}

#endif
