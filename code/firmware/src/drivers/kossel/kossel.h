#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H

#include "drivers/driver.h"
#include "drivers/axisstepper.h"
#include "drivers/extruderstepper.h"
#include "drivers/linearstepper.h"
#include "drivers/lineardeltastepper.h"
#include "drivers/rpi/a4988.h"
#include "drivers/rpi/sn754410.h"
#include "drivers/linearcoordmap.h"
#include "drivers/lineardeltacoordmap.h"
#include "drivers/rpi/onepinenabler.h"
#include "drivers/rpi/leverendstop.h"
#include <tuple>

//R1000 = distance from (0, 0) (platform center) to each axis, in micrometers (1e-6)
//L1000 = length of the rods that connect each axis to the end effector
//STEPS_M = #of steps for the motor driving each axis (A, B, C) to raise its carriage by 1 meter.
#define R1000 100000
#define L1000 260000
#define H1000 800000
#define STEPS_M 9000
#define STEPS_M_EXT 1000

namespace drv {

class Kossel : public Driver {
	private:
		typedef rpi::OnePinEnabler<RPI_V2_GPIO_P1_16, 0> _StepperEn; //enable pin is LOW for on, HIGH for off
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_18, 0, BCM2835_GPIO_PUD_DOWN> _EndstopA; //endstop is triggered on HIGH
    public:
        //typedef std::tuple<LinearStepper<10000, COORD_X>, LinearStepper<1000, COORD_Y>, LinearStepper<1000, COORD_Z>, LinearStepper<1000, COORD_E> > AxisStepperTypes;
        typedef LinearDeltaCoordMap<0, 1, 2, 3, R1000, L1000, H1000, STEPS_M> CoordMapT;
        typedef std::tuple<LinearDeltaStepper<0, CoordMapT, R1000, L1000, STEPS_M, _EndstopA>, LinearDeltaStepper<1, CoordMapT, R1000, L1000, STEPS_M, _EndstopA>, LinearDeltaStepper<2, CoordMapT, R1000, L1000, STEPS_M, _EndstopA>, LinearStepper<STEPS_M_EXT, COORD_E> > AxisStepperTypes;
        typedef std::tuple<
        	//rpi::SN754410<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_16, RPI_V2_GPIO_P1_18>, //X coord
        	rpi::A4988<RPI_V2_GPIO_P1_19, RPI_V2_GPIO_P1_21, _StepperEn>, //X coord
        	rpi::A4988<RPI_V2_GPIO_P1_22, RPI_V2_GPIO_P1_23, _StepperEn>, //Y coord
        	rpi::A4988<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, _StepperEn>, //Z coord
        	rpi::A4988<RPI_V2_GPIO_P1_11, RPI_V2_GPIO_P1_12, _StepperEn>  > IODriverTypes; //E coord
        //typedef LinearCoordMap<0, 1, 2, 3> CoordMapT; //map A->X, B->Y, C->Z, D->E
        IODriverTypes ioDrivers;
        _EndstopA endstops;
        constexpr static std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        void getTemperature(int &extruder, int& platform) const;
        inline float defaultMoveRate() const { //in mm/sec
        	return 50;
        }
};

}

#endif
