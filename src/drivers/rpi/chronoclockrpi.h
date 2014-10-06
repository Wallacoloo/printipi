#ifndef DRIVERS_RPI_CHRONOCLOCKRPI_H
#define DRIVERS_RPI_CHRONOCLOCKRPI_H

#include <chrono> //for std::chrono::*
#include "rpi.h"
#include "bcm2835.h"


namespace drv {
namespace rpi {


class ChronoClockRpi {
    static InitRpiType _i; //ensure rpi is init before any calls to now() occur.
    public:
        typedef std::chrono::microseconds duration;
        typedef duration::rep rep;
        typedef duration::period period;
        typedef std::chrono::time_point<ChronoClockRpi> time_point;
        static const bool is_steady = true;
        inline static time_point now() noexcept {
            //struct timespec tnow = timespecNow();
            return time_point(std::chrono::microseconds(bcm2835_st_read()));
            /*struct timespec tnow;
            clock_gettime(CLOCK_MONOTONIC, &tnow);
            return time_point(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(tnow.tv_sec)) 
                + std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::nanoseconds(tnow.tv_nsec))
            );// + std::chrono::nanoseconds(tnow.tv_nsec);*/
        }
};


}
}

#endif
