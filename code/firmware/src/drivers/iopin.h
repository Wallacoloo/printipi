#ifndef DRIVERS_IOPIN_H
#define DRIVERS_IOPIN_H

#include "common/logging.h"

namespace drv {

enum IoLevel {
	IoLow = 0,
	IoHigh = 1
};

template <typename ThisT, IoLevel lev=IoLow> class IoPinOnExit {
	IoPinOnExit() {
		//LOG("IoPinOnExit init\n");
		printf("IoPinOnExit init\n");
		SchedulerBase::registerExitHandler((void(*)())&deactivate, SCHED_IO_EXIT_LEVEL);
	}
	static void deactivate() {
		LOGV("IoPinOnExit::deactivate\n");
		ThisT pin;
		pin.digitalWrite(lev);
	}
};

//IoPin defines the interface for a GPIO pin, as well as default implementations of each function in case they aren't supported by the actual driver. Each microcontroller platform should provide its own IoPin implementation that inherits from this class.
struct IoPin {
	//template <typename ThisT> IODriver(const ThisT * /* _this */) {
	//	SchedulerBase::registerExitHandler((void(*)())&ThisT::digitalWrite, SCHED_IO_EXIT_LEVEL);
	//}
	//configure the pin as an output. Many microcontrollers have GPIOs. Before writing an output value to them, they must first be configured as an output. But we also want the initial state of the pin to be defined, if possible.
	inline void makeDigitalOutput(IoLevel /*lev*/) {}
	//configure the pin to be an input
	inline void makeDigitalInput() {}
	//read the pin's input value (assumes pin is configured as digital)
	inline IoLevel digitalRead() const { return IoLow; }
	inline void digitalWrite(IoLevel /*lev*/) {}
};

//easy way to proxy reads/writes, but invert them (eg Inverted<IoPin>.digitalWrite(IoHigh) really calls IoPin.digitalWrite(IoLow), etc)
template <typename Pin, bool InvertWrite=true, bool InvertRead=true> class InvertedPin : public IoPin {
	Pin _pin;
	IoLevel invertLev(IoLevel lev, bool doInvert) const {
		return doInvert ? (lev == IoHigh ? IoLow : IoHigh) : lev;
	}
	public:
		void makeDigitalOutput(IoLevel lev) {
			_pin.makeDigitalOutput(invertLev(lev, InvertWrite));
		}
		inline void makeDigitalInput() {
			_pin.makeDigitalInput();
		}
		inline IoLevel digitalRead() const {
			return invertLev(_pin.digitalRead(), InvertRead);
		}
		inline void digitalWrite(IoLevel lev) {
			_pin.digitalWrite(invertLev(lev, InvertWrite));
		}
};

//default implementation of IoPin (does nothing):
struct NoPin : public IoPin {
};

}
#endif
