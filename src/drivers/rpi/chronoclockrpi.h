#ifndef DRIVERS_RPI_CHRONOCLOCKRPI_H
#define DRIVERS_RPI_CHRONOCLOCKRPI_H

#include <chrono> //for std::chrono::*
#include "mitpi.h"


namespace drv {
namespace rpi {


class ChronoClockRpi {
    static mitpi::InitMitpiType _i; //ensure mitpi is init before any calls to now() occur.
    public:
        typedef std::chrono::microseconds duration;
        typedef duration::rep rep;
        typedef duration::period period;
        typedef std::chrono::time_point<ChronoClockRpi> time_point;
        static const bool is_steady = true;
        inline static time_point now() noexcept {
            //struct timespec tnow = timespecNow();
            //return time_point(std::chrono::microseconds(bcm2835_st_read()));
            return time_point(std::chrono::microseconds(mitpi::readSysTime()));
        }
};


}
}

#endif
