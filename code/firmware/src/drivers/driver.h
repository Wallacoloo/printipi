#ifndef DRIVERS_DRIVER_H
#define DRIVERS_DRIVER_H

/* 
 * Printipi/drivers/driver.h
 * (c) 2014 Colin Wallace
 *
 * Drivers mix and match the various interface implementations in order to define the machine as a whole.
 * see src/drivers/kossel/kossel.h for an example.
 * Each driver needs to provide a CoordMap, some AxisSteppers, and some IODrivers, as well as a few simple functions.
 * It is within the driver file that the machine-specific settings (IO ports, build volume, temperature PID values, etc) are defined.
 *
 * Note: Driver is an interface, and not an implementation.
 * An implementation is needed for each individual machine model (eg marlin, replicator 2, kossel mini).
 * These implementations must provide the functions and typedefs outlined further down in the header.
 */

#include <cstddef> //for size_t
#include "event.h" //for gparse::StepDirection

namespace drv {

class Driver {
	public:
		//void getTemperature(int &extruder, int& platform) const;
		//constexpr std::size_t numAxis() const;
		//static std::tuple<AxisStepperA, AxisStepperB, ...> AxisSteppers;
		//virtual float defaultMoveRate() const = 0;
};

}
#endif
