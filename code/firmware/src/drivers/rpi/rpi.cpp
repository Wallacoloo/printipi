#include "rpi.h"
#include "bcm2835.h"

#include <cstdlib> //for atexit

namespace drv {
namespace rpi {

bool wasBcmInit = false;

void atexit_bcm2835_close() {
	bcm2835_close(); //return value is int.
}

void initIO() {
	if (!wasBcmInit) {
		wasBcmInit = true;
		//wiringPiSetup();
		bcm2835_init();
		std::atexit(atexit_bcm2835_close);
		//atexit: bcm2835_close
	}
}

}
}
