#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H

#include "drivers/driver.h"
#include "drivers/axisstepper.h"
#include "drivers/extruderstepper.h"
#include "drivers/linearstepper.h"
#include <tuple>

namespace drv {

class Kossel : public Driver {
    public:
        void getTemperature(int &extruder, int& platform) const;
        typedef std::tuple<LinearStepper<20000, COORD_X>, LinearStepper<20000, COORD_Y>, LinearStepper<20000, COORD_Z>, LinearStepper<20000, COORD_E> > AxisSteppers;
        typedef std::tuple<> IODrivers;
        constexpr static std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        float defaultMoveRate() const;
        float defaultFeedRate() const;
};

}

#endif
