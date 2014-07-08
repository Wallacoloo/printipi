#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H

#include "drivers/driver.h"
#include "drivers/axisstepper.h"
#include "drivers/extruderstepper.h"
#include "drivers/linearstepper.h"
#include "drivers/rpi/a4988.h"
#include <tuple>

namespace drv {

class Kossel : public Driver {
    public:
        typedef std::tuple<LinearStepper<20000, COORD_X>, LinearStepper<20000, COORD_Y>, LinearStepper<20000, COORD_Z>, LinearStepper<20000, COORD_E> > AxisSteppers;
        typedef std::tuple<
        	rpi::A4988<RPI_GPIO_P1_11, RPI_GPIO_P1_12>,
        	rpi::A4988<RPI_GPIO_P1_11, RPI_GPIO_P1_12>,
        	rpi::A4988<RPI_GPIO_P1_11, RPI_GPIO_P1_12>,
        	rpi::A4988<RPI_GPIO_P1_11, RPI_GPIO_P1_12>  > IODriverTypes;
        IODriverTypes ioDrivers;
        constexpr static std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        void getTemperature(int &extruder, int& platform) const;
        float defaultMoveRate() const;
        float defaultFeedRate() const;
};

}

#endif
