#ifndef DRIVERS_MACHINES_MACHINE_H
#define DRIVERS_MACHINES_MACHINE_H

namespace drv {

class Machine {
	public:
		inline float defaultMoveRate() const { //in mm/sec
        	return 0;
        }
        //currently have to be satisfied with mins/maxes - can't achieve more without muddying the interface, and I see little reason for having more.
        inline float maxRetractRate() const { //in mm/sec
        	return 0;
        }
        inline float maxExtrudeRate() const { //in mm/sec
        	return 0;
        }
        inline float clampMoveRate(float inp) const {
        	return inp; 
        }
        inline float clampHomeRate(float inp) const {
        	return inp;
        }
        inline bool doHomeBeforeFirstMovement() const {
        	return true; //if we get a G1 before the first G28, then yes - we want to home first.
        }
};

}
#endif
