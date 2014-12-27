#ifndef PLATFORMS_GENERIC_PRIMITIVEIOPIN_H
#define PLATFORMS_GENERIC_PRIMITIVEIOPIN_H

#include <tuple>

#include "compileflags.h" //for IoLevel
#include "common/logging.h"

namespace plat {
namespace generic {

class PrimitiveIoPin {
	//Implementation for a basic (do-nothing) GPIO pin
	public:
		inline static PrimitiveIoPin null() { return PrimitiveIoPin(); }
		inline bool isNull() const { return true; }
		//We want to use this PrimitiveIoPin on any architecture, 
		// so let it be constructed with whatever platform-specific arguments the config file uses with its IO pins
		template <typename ...T> PrimitiveIoPin(T ...args) {
			(void)std::make_tuple(args...); //unused
		}

		//do-nothing implementations for basic functions
		//Note: the return type of id() is platform-specific, though it must never be void.
		inline int id() const { return -1; }
		inline void makeDigitalOutput(IoLevel) {
			LOGW_ONCE("Attempt to makeDigitalOutput() the generic PrimitiveIoPin interface\n");
		}
	    //configure the pin to be an input
	    inline void makeDigitalInput() {
	    	LOGW_ONCE("Attempt to makeDigitalInput() the generic PrimitiveIoPin interface\n");
	    }
	    //read the pin's input value (assumes pin is configured as digital)
	    inline IoLevel digitalRead() const { 
	    	LOGW_ONCE("Attempt to digitalRead() the generic PrimitiveIoPin interface\n");
	    	return IoLow; 
	    }
	    inline void digitalWrite(IoLevel) {
	    	LOGW_ONCE("Attempt to digitalWrite() the generic PrimitiveIoPin interface\n");
	    }
};

}
}


#endif