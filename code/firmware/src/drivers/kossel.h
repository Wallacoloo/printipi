#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H

#include "driver.h"

namespace drv {

class Kossel : public Driver {
    public:
        void getTemperature(int &extruder, int& platform) const;
        std::size_t numAxis() const;
        float defaultMoveRate() const;
        float defaultFeedRate() const;
        float relativeTimeOfNextStep(int axisIdx, float &x, float &y, float &z, float &e, float velx, float vely, float velz, float velExt) const;
};

}

#endif
