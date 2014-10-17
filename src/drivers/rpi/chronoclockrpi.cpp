//Only want to initialize mitpi if we are targeting the raspberry pi:
#ifdef TARGET_RPI

#include "chronoclockrpi.h"

namespace drv {
namespace rpi {

static mitpi::InitMitpiType ChronoClockRpi::_i;

}
}

#endif
