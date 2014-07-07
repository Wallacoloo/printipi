#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H

#include "driver.h"
#include "axisstepper.h"
#include <tuple>

namespace drv {

class Kossel : public Driver {
    public:
        void getTemperature(int &extruder, int& platform) const;
        typedef std::tuple<AxisStepper, AxisStepper, AxisStepper, AxisStepper> AxisSteppers;
        constexpr static std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        float defaultMoveRate() const;
        float defaultFeedRate() const;
        //float relativeTimeOfNextStep(int axisIdx, StepDirection &dir, float x, float y, float z, float e, float velx, float vely, float velz, float velExt) const;
};

}

#endif
