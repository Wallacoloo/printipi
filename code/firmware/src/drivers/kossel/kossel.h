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
#include <tuple>

//R1000 = distance from (0, 0) (platform center) to each axis, in micrometers (1e-6)
//L1000 = length of the rods that connect each axis to the end effector
//STEPS_M = #of steps for the motor driving each axis (A, B, C) to raise its carriage by 1 meter.
#define R1000 100000
#define L1000 260000
#define STEPS_M 10000
#define STEPS_M_EXT 1000

namespace drv {

class Kossel : public Driver {
    public:
        //typedef std::tuple<LinearStepper<10000, COORD_X>, LinearStepper<1000, COORD_Y>, LinearStepper<1000, COORD_Z>, LinearStepper<1000, COORD_E> > AxisStepperTypes;
        typedef LinearDeltaCoordMap<0, 1, 2, 3, R1000, L1000, STEPS_M> CoordMapT;
        typedef std::tuple<LinearDeltaStepper<0, CoordMapT, R1000, L1000, STEPS_M>, LinearDeltaStepper<1, CoordMapT, R1000, L1000, STEPS_M>, LinearDeltaStepper<2, CoordMapT, R1000, L1000, STEPS_M>, LinearStepper<STEPS_M_EXT, COORD_E> > AxisStepperTypes;
        typedef std::tuple<
        	//rpi::A4988<RPI_GPIO_P1_11, RPI_GPIO_P1_12>,
        	rpi::SN754410<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_16, RPI_V2_GPIO_P1_18>, //X coord
        	rpi::A4988<RPI_V2_GPIO_P1_11, RPI_V2_GPIO_P1_12>, //Y coord
        	rpi::A4988<RPI_V2_GPIO_P1_11, RPI_V2_GPIO_P1_12>, //Z coord
        	rpi::A4988<RPI_V2_GPIO_P1_11, RPI_V2_GPIO_P1_12>  > IODriverTypes;
        //typedef LinearCoordMap<0, 1, 2, 3> CoordMapT; //map A->X, B->Y, C->Z, D->E
        IODriverTypes ioDrivers;
        constexpr static std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        void getTemperature(int &extruder, int& platform) const;
        float defaultMoveRate() const;
};

}

#endif
