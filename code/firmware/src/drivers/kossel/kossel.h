#ifndef DRIVERS_KOSSEL_H
#define DRIVERS_KOSSEL_H


#include "pid.h"
#include "filters/lowpassfilter.h"
#include "drivers/driver.h"
#include "drivers/axisstepper.h"
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
//#define R1000 125000
//#define L1000 215000
//#define H1000 507000
#define R1000 121000
#define L1000 222000
#define H1000 518700
//#define STEPS_M 9000
#define STEPS_M 5200 //no microstepping enabled.
#define STEPS_M_EXT 4000

#define THERM_RA 665
#define THERM_CAP_PICO 100000
#define VCC_mV 3300
#define THERM_IN_THRESH_mV 1600
#define THERM_T0 25
#define THERM_R0 100000
#define THERM_BETA 3950

/*Used IOs:
  3
  5
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
/* Calibrating:
  as y leaves 0 to +side, z increases (should stay level)
    Even more so as it goes to -side.
  as x becomes +size, z increases
  as x becomes -size, z increases
  This points to either R or L being off, but in what way?
    joint to edge of bed is ~43 mm. bed is 170mm, so R is 43 + 85 = 128mm
    L is 215mm as measured BUT math doesn't consider the existence of an effector (so L should be longer?)
  Note: increasing L increases convexity (/\)
  Note: increasing R increases concavity (\/)
  Note: decreasing R increases convexity (/\)
*/
namespace drv {

class Kossel : public Driver {
	private:
		typedef rpi::OnePinEnabler<RPI_V2_GPIO_P1_16, 0> _StepperEn; //enable pin is LOW for on, HIGH for off
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_18, LOW, BCM2835_GPIO_PUD_DOWN> _EndstopA; //endstop is triggered on HIGH
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_24, LOW, BCM2835_GPIO_PUD_DOWN> _EndstopB;
		typedef rpi::LeverEndstop<RPI_V2_GPIO_P1_26, LOW, BCM2835_GPIO_PUD_DOWN> _EndstopC;
		typedef rpi::RCThermistor<RPI_V2_GPIO_P1_07, THERM_RA, THERM_CAP_PICO, VCC_mV, THERM_IN_THRESH_mV, THERM_T0, THERM_R0, THERM_BETA> _Thermistor;
		typedef rpi::OnePinIODriver<RPI_V2_GPIO_P1_08, 1> _Fan;
		typedef rpi::OnePinIODriver<RPI_V2_GPIO_P1_10, 0> _HotendOut;
		//typedef TempControl<_HotendOut, _Thermistor> _HotendController;
    public:
        //typedef std::tuple<LinearStepper<10000, COORD_X>, LinearStepper<1000, COORD_Y>, LinearStepper<1000, COORD_Z>, LinearStepper<1000, COORD_E> > AxisStepperTypes;
        typedef LinearDeltaCoordMap</*0, 1, 2, 3, */ R1000, L1000, H1000, STEPS_M, STEPS_M_EXT> CoordMapT;
        typedef std::tuple<LinearDeltaStepper<0, CoordMapT, R1000, L1000, STEPS_M, _EndstopA>, LinearDeltaStepper<1, CoordMapT, R1000, L1000, STEPS_M, _EndstopB>, LinearDeltaStepper<2, CoordMapT, R1000, L1000, STEPS_M, _EndstopC>, LinearStepper<STEPS_M_EXT, COORD_E> > AxisStepperTypes;
        typedef std::tuple<
        	//rpi::SN754410<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_16, RPI_V2_GPIO_P1_18>, //X coord
        	//rpi::A4988<RPI_V2_GPIO_P1_19, RPI_V2_GPIO_P1_21, _StepperEn>, //A tower
        	//rpi::A4988<RPI_V2_GPIO_P1_22, RPI_V2_GPIO_P1_23, _StepperEn>, //B tower
        	//rpi::A4988<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, _StepperEn>, //C tower
        	rpi::A4988<RPI_V2_GPIO_P1_22, RPI_V2_GPIO_P1_23, _StepperEn>, //A tower
        	rpi::A4988<RPI_V2_GPIO_P1_19, RPI_V2_GPIO_P1_21, _StepperEn>, //B tower
        	rpi::A4988<RPI_V2_GPIO_P1_13, RPI_V2_GPIO_P1_15, _StepperEn>, //C tower
        	rpi::A4988<RPI_V2_GPIO_P1_03, RPI_V2_GPIO_P1_05, _StepperEn>, //E coord
        	_Fan,
        	//_HotendController,
        	//12000, 3000, 1000 gives osc of ~3 min (20C-80C). Converges.
        	//20000,  600,    0 (50C->80C). Converges. No osc. Takes 2 minutes to progress from 81C to 80C. Peaks at 130C when from (80C->120C). Critically damped. Takes 90 seconds to stabilize *near* target.
        	//12000,  600, 1200 (50C->130C). Peaks 22C above target. Underdamped. 5 mins to converge
        	//18000,  300, 1000 (40C->130C). Overdamped. 4.5 minutes to reach target (& is stabilized when it gets there)
        	TempControl<5, _HotendOut, _Thermistor, PID<18000, 300, 1000>, LowPassFilter<3000> >,
        	_EndstopA, _EndstopB, _EndstopC
        	> IODriverTypes;
        //typedef LinearCoordMap<0, 1, 2, 3> CoordMapT; //map A->X, B->Y, C->Z, D->E
        IODriverTypes ioDrivers;
        //std::tuple<_EndstopA, _EndstopB, _EndstopC> _endstops;
        //_Thermistor thermistor;
        inline AxisIdType getFanIODriverIdx() const {
        	return 4;
        }
        inline float defaultFanPwmPeriod() const {
        	return 0.02; //don't need high resolution
        }
        inline std::tuple<CelciusType, CelciusType> getTemperature() const {
        	return std::make_tuple(std::get<5>(ioDrivers).getLastTemp(), -300);
        }
        inline void setTemperature(CelciusType temp) {
        	std::get<5>(ioDrivers).setTemp(temp);
        }
        inline float defaultMoveRate() const { //in mm/sec
        	return 30;
        }
        inline float maxAccel() const { //in mm/sec
        	return 600;
        }
        inline float clampMoveRate(float inp) const {
        	return std::min(inp, defaultMoveRate());//ensure we never move too fast.
        }
        inline float clampHomeRate(float /*inp*/) const {
        	return 10;
        }
        inline float clampExtrusionRate(float rate) const {
        	//need to cover both the positive (extruding) and negative (retracting) possibilities.
        	return std::max((float)-10, std::min(rate, (float)10));
        }
};

}

#endif
