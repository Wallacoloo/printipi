//Only want to initialize mitpi if we are targeting the raspberry pi:
#include "common/typesettings/compileflags.h"
#ifdef TARGET_RPI

#include "chronoclock.h"

namespace drv {
namespace rpi {

static mitpi::InitMitpiType ChronoClock::_i;

}
}

#endif
