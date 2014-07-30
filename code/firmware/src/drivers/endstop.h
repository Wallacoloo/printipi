#ifndef DRIVERS_ENDSTOP_H
#define DRIVERS_ENDSTOP_H

/* 
 * Printipi/drivers/axisstepper.h
 * (c) 2014 Colin Wallace
 *
 * Endstops are queriable switches placed at the axis limits.
 * They typically represent a "known" point to which the device can be homed upon initiailization,
 *   or a point beyond which the device should not be pushed.
 *
 * Note: Endstop is an interface, and not an implementation.
 * An implementation is needed for each physical endstop - X axis, Y axis, etc.
 * These implementations must provide the functions outlined further down in the header.
 */

#include "iodriver.h"

namespace drv {

class Endstop : public IODriver {
	public:
		template <typename ThisT> Endstop(ThisT *_this) : IODriver(_this) {}
		inline static bool isTriggered() { return false; }
};

//default implementation for an axis which doesn't have an endstop.
class EndstopNoExist : public Endstop {
	public:
		EndstopNoExist() : Endstop(this) {}
};

}
#endif
