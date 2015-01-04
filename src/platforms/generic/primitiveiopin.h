/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef PLATFORMS_GENERIC_PRIMITIVEIOPIN_H
#define PLATFORMS_GENERIC_PRIMITIVEIOPIN_H

#include <tuple>

#include "compileflags.h" //for IoLevel
#include "common/logging.h"

namespace plat {
namespace generic {

//Implementation for a basic (do-nothing) GPIO pin
class PrimitiveIoPin {
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
		inline void makeDigitalOutput(IoLevel level) {
			(void)level;
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
	    //Write a digital value to the pin. Note: must first call makeDigitalOutput.
	    inline void digitalWrite(IoLevel level) {
	    	(void)level;
	    	LOGW_ONCE("Attempt to digitalWrite() the generic PrimitiveIoPin interface\n");
	    }
};

}
}


#endif