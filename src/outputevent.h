#ifndef OUTPUTEVENT_H
#define OUTPUTEVENT_H

/* 
 * Printipi/outputevent.h
 * (c) 2014 Colin Wallace
 *
 * An OutputEvent encapsulates information about the desired state for a GPIO pin at a given time.
 *   Eg, "set pin (2) (high) at t=(1234567) uS", or "set pin (44) (low) at t=(887766) uS"
 */
 
#include "common/typesettings/clocks.h" //for EventClockT
#include "common/typesettings/primitives.h" //for GpioPinIdType
//typedef int GpioPinIdType;

class OutputEvent {
    EventClockT::time_point _time;
	GpioPinIdType _pinId;
	bool _state; //1=HIGH, 0=LOW
	public:
	    OutputEvent(EventClockT::time_point time, GpioPinIdType pinId, bool state) : _time(time), _pinId(pinId), _state(state) {
	    }
	    inline EventClockT::time_point time() const {
	        return _time;
	    }
	    inline GpioPinIdType pinId() const {
	        return _pinId;
	    }
	    inline bool state() const {
	        return _state;
	    }
};


#endif
