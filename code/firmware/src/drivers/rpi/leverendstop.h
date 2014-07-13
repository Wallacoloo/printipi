#ifndef DRIVERS_RPI_LEVERENDSTOP_H
#define DRIVERS_RPI_LEVERENDSTOP_H

#include "drivers/endstop.h"
#include "drivers/rpi/rpi.h"

namespace drv {
namespace rpi {


template <uint8_t Pin, bool ValueTriggered> class LeverEndstop : public Endstop {
	public:
};


}
}
#endif
