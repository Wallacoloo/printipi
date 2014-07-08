#include "rpi.h"
#include "bcm2835.h"

namespace drv {
namespace rpi {

bool wasBcmInit = false;

void initIO() {
	if (!wasBcmInit) {
		wasBcmInit = true;
		//wiringPiSetup();
		bcm2835_init();
		//atexit: bcm2835_close
	}
}

}
}
