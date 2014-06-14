#ifndef DRIVER_H
#define DRIVER_H

class Driver {
	public:
		//M105
		void getTemperature(int &extruder, int& platform);
};

#endif
