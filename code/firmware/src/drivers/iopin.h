#ifndef DRIVERS_IOPIN_H
#define DRIVERS_IOPIN_H

namespace drv {

enum IoLevel {
	IoLow = 0,
	IoHigh = 1
};

struct IoPin {
	//configure the pin as an output. Many microcontrollers have GPIOs. Before writing an output value to them, they must first be configured as an output. But we also want the initial state of the pin to be defined, if possible.
	inline void makeDigitalOutput(IoLevel /*lev*/) {}
	//configure the pin to be an input
	inline void makeDigitalInput() {}
	//read the pin's input value (assumes pin is configured as digital)
	inline IoLevel digitalRead() const { return IoLow; }
	inline void digitalWrite(IoLevel /*lev*/) {}
};


}
#endif
