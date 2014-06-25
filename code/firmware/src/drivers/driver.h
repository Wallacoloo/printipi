#ifndef DRIVERS_DRIVER_H
#define DRIVERS_DRIVER_H

namespace drv {

class Driver {
	public:
		//M105
		void getTemperature(int &extruder, int& platform);
};

}
#endif
