#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H


#include "pid.h"
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
#include "drivers/rpi/rcthermistor.h"
#include "drivers/rpi/onepiniodriver.h"
#include "drivers/tempcontrol.h"
#include <tuple>

//R1000 = distance from (0, 0) (platform center) to each axis, in micrometers (1e-6)
//L1000 = length of the rods that connect each axis to the end effector
//STEPS_M = #of steps for the motor driving each axis (A, B, C) to raise its carriage by 1 meter.
#define R1000 125000
#define L1000 215000
#define H1000 500000
//#define STEPS_M 9000
#define STEPS_M 4500 //no microstepping enabled.
#define STEPS_M_EXT 1000

#define THERM_RA 665
#define THERM_CAP_PICO 100000
#define VCC_mV 3300
#define THERM_IN_THRESH_mV 1600
#define THERM_T0 25
#define THERM_R0 100000
#define THERM_BETA 3950

/*Used IOs:
  7
  8
  10
  (11)
  (12)
  13
  15
  16
  18
  19
  21
  22
  23
  24
  26
*/
namespace drv {

class Kossel : public Driver {
	private:
		typedef rpi::OnePinEnabler<RPI_V2_GPIO_P1_16, 0> _StepperEn; //enable pin is LOW for on, HIGH for off
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_18, 0, BCM2835_GPIO_PUD_DOWN> _EndstopA; //endstop is triggered on HIGH
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_24, 0, BCM2835_GPIO_PUD_DOWN> _EndstopB;
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_26, 0, BCM2835_GPIO_PUD_DOWN> _EndstopC;
		typedef rpi::RCThermistor<RPI_V2_GPIO_P1_07, THERM_RA, THERM_CAP_PICO, VCC_mV, THERM_IN_THRESH_mV, THERM_T0, THERM_R0, THERM_BETA> _Thermistor;
		typedef rpi::OnePinIODriver<RPI_V2_GPIO_P1_08, 1> _Fan;
		typedef rpi::OnePinIODriver<RPI_V2_GPIO_P1_10, 0> _HotendOut;
		//typedef TempControl<_HotendOut, _Thermistor> _HotendController;
    public:
        //typedef std::tuple<LinearStepper<10000, COORD_X>, LinearStepper<1000, COORD_Y>, LinearStepper<1000, COORD_Z>, LinearStepper<1000, COORD_E> > AxisStepperTypes;
        typedef LinearDeltaCoordMap<0, 1, 2, 3, R1000, L1000, H1000, STEPS_M> CoordMapT;
        typedef std::tuple<LinearDeltaStepper<0, CoordMapT, R1000, L1000, STEPS_M, _EndstopA>, LinearDeltaStepper<1, CoordMapT, R1000, L1000, STEPS_M, _EndstopB>, LinearDeltaStepper<2, CoordMapT, R1000, L1000, STEPS_M, _EndstopC>, LinearStepper<STEPS_M_EXT, COORD_E> > AxisStepperTypes;
        typedef std::tuple<
        	//rpi::SN754410<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_16, RPI_V2_GPIO_P1_18>, //X coord
        	rpi::A4988<RPI_V2_GPIO_P1_19, RPI_V2_GPIO_P1_21, _StepperEn>, //A tower
        	rpi::A4988<RPI_V2_GPIO_P1_22, RPI_V2_GPIO_P1_23, _StepperEn>, //B tower
        	rpi::A4988<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, _StepperEn>, //C tower
        	rpi::A4988<RPI_V2_GPIO_P1_11, RPI_V2_GPIO_P1_12, _StepperEn>, //E coord
        	_Fan,
        	//_HotendController,
        	//12000, 3000, 1000 gives osc of ~3 min. Converges.
        	TempControl<5, _HotendOut, _Thermistor, PID<20000, 600, 0> >,
        	_EndstopA, _EndstopB, _EndstopC
        	> IODriverTypes;
        //typedef LinearCoordMap<0, 1, 2, 3> CoordMapT; //map A->X, B->Y, C->Z, D->E
        IODriverTypes ioDrivers;
        //std::tuple<_EndstopA, _EndstopB, _EndstopC> _endstops;
        //_Thermistor thermistor;
        constexpr static std::size_t numAxis() {
            return 4; //A, B, C + Extruder
        }
        inline AxisIdType getFanIODriverIdx() const {
        	return 4;
        }
        inline float defaultFanPwmPeriod() const {
        	return 0.02; //don't need high resolution
        }
        inline void getTemperature(CelciusType &extruder, CelciusType& /*platform*/) const {
        	extruder = std::get<5>(ioDrivers).getLastTemp();
        	//extruder = thermistor.readTemperature(); // *100000;
        }
        inline void setTemperature(CelciusType temp) {
        	std::get<5>(ioDrivers).setTemp(temp);
        }
        inline float defaultMoveRate() const { //in mm/sec
        	return 30;
        }
        inline float clampMoveRate(float inp) const {
        	if (inp > defaultMoveRate()) { //ensure we never move too fast.
        		return defaultMoveRate();
        	}
        	return inp;
        }
        inline float clampHomeRate(float /*inp*/) const {
        	return 10;
        }
};

}

#endif
