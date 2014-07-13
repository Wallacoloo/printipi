#ifndef DRIVERS_ENDSTOP_H
#define DRIVERS_ENDSTOP_H

namespace drv {

class Endstop : public IODriver {
	public:
		inline bool isTriggered() { return false; }
};

}
#endif
