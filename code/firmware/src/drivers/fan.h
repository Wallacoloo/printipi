#ifndef DRIVERS_FAN_H
#define DRIVERS_FAN_H

/* 
 * Printipi/drivers/axisstepper.h
 * (c) 2014 Colin Wallace
 *
 * The Fan class serves to control a physical Fan, often used to cool cpu components.
 * This class essentially wraps another IO driver (like /src/drivers/rpi/onepiniodriver.h) so that it can be commanded as a fan
 */

namespace drv {

template <typename IODrive> class Fan : public IODriver {


};

}
#endif
