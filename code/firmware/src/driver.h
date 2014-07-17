#ifndef DRIVER_H
#define DRIVER_H

class Driver {
	public:
		//M105
		void getTemperature(CelciusType &extruder, CelciusType& platform);
};

#endif
