#ifndef DRIVERS_ENDSTOP_H
#define DRIVERS_ENDSTOP_H

#include "iodriver.h"

namespace drv {

class Endstop : public IODriver {
	public:
		template <typename ThisT> Endstop(ThisT *_this) : IODriver(_this) {}
		inline static bool isTriggered() { return false; }
};

class EndstopNoExist : public Endstop {
	public:
		EndstopNoExist() : Endstop(this) {}
};

}
#endif
