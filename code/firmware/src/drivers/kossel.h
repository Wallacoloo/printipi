#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H

#include "driver.h"

namespace drv {

class Kossel : public Driver {
    public:
        void getTemperature(int &extruder, int& platform);
};

}

#endif
