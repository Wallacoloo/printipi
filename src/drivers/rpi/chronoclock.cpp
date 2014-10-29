//Only want to initialize mitpi if we are targeting the raspberry pi:
#include "compileflags.h"
#ifdef DTARGET_PLATFORM_RPI

#include "chronoclock.h"

namespace drv {
namespace rpi {

mitpi::InitMitpiType ChronoClock::_i;

}
}

#endif
