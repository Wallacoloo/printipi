#ifndef DRIVERS_RPI_RPI_H
#define DRIVERS_RPI_RPI_H

namespace drv {
namespace rpi {

void initIO();

//allow to initialize RPi via instantiation of a class.
//Useful for static initializers:
struct InitRpiType {
	inline InitRpiType() {
		initIO();
	}
};

}
}

#endif
