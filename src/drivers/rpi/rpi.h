#ifndef DRIVERS_RPI_RPI_H
#define DRIVERS_RPI_RPI_H

/* 
 * Printipi/drivers/rpi/rpi.h
 * (c) 2014 Colin Wallace
 * 
 * Responsible for initializing & unloading the bcm2835 library (src/drivers/rpi/bcm2835.h)
 */

namespace drv {
namespace rpi {

void initIO();

//allow to initialize RPi via instantiation of a class.
//Useful for static initializers:
struct InitRpiType {
    inline InitRpiType() {
        initIO();
    }
};

}
}

#endif
