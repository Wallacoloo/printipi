//Only want to initialize mitpi if we are targeting the raspberry pi:
#include "common/typesettings/compileflags.h"
#ifdef TARGET_RPI

#include "chronoclockrpi.h"

namespace drv {
namespace rpi {

static mitpi::InitMitpiType ChronoClockRpi::_i;

}
}

#endif
