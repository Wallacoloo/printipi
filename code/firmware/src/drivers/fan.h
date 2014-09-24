#ifndef DRIVERS_FAN_H
#define DRIVERS_FAN_H

/* 
 * Printipi/drivers/fan.h
 * (c) 2014 Colin Wallace
 *
 * The Fan class serves to control a physical Fan, often used to cool cpu components.
 * This class essentially wraps an IoPin so that it can be commanded as a fan
 */
 
#include "iodriver.h"

#include <tuple>

namespace drv {

template <typename Driver> class Fan : public IODriver {
    Driver driver;
    public:
        Fan() : IODriver() {
            driver.makeDigitalOutput(IoLow);
        }
        constexpr bool isFan() { return true; }
        //forward output control:
        void stepForward() {
            driver.digitalWrite(IoHigh);
        }
        void stepBackward() {
            driver.digitalWrite(IoLow);
        }
        bool canDoPwm() const {
            return true;
        }
        Driver& getPwmPin() { //Note: will be able to handle PWMing multiple pins, too, if one were just to use a wrapper and pass it as the Driver type.
            return driver;
        }
};

}
#endif
