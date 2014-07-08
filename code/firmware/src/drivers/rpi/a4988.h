#ifndef DRIVERS_RPI_A4988
#define DRIVERS_RPI_A4988

#include "rpi.h"

namespace drv {
namespace rpi {

template <int STEPPIN, int DIRPIN> class A4988 : public IODriver {
	public:
		A4988() {
			initIO();
		}
};


}
}

#endif
