#include "rpi.h"

namespace drv {
namespace rpi {

bool wasWiringInit = false;

void initIO() {
	if (!wasWiringInit) {
		wasWiringInit = true;
		wiringPiSetup();
	}
}

}
}
