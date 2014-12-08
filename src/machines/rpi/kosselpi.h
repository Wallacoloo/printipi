#ifndef DRIVERS_KOSSELPI_H
#define DRIVERS_KOSSELPI_H


#include "machines/machine.h"
#include "pid.h"
#include "common/filters/lowpassfilter.h"
#include "common/matrix.h"
#include "motion/constantacceleration.h"
#include "drivers/linearstepper.h"
#include "drivers/lineardeltastepper.h"
#include "drivers/rpi/rpiiopin.h"
#include "drivers/a4988.h"
#include "drivers/lineardeltacoordmap.h"
#include "drivers/rcthermistor.h"
#include "drivers/tempcontrol.h"
#include "drivers/fan.h"
#include "drivers/rpi/mitpi.h" //for pin numberings
#include <tuple>

//All of the #defines between this point and the end of this file are ONLY used within this file,

//Build calibration settings:
#define R_MM 111.000                // Distance from the center of the build plate to each delta tower, in mm
#define L_MM 221.000                // Length of the vertical rods that connect the carriages to the end effector, in mm
#define H_MM 467.450                // The vertical distance from your endstops to the build platform, in mm
#define BUILDRAD_MM 85.000          // The radius of your printbed, in mm (used to ensure desired coordinates are valid)
#define STEPS_MM 6.265*8            // Number of stepper motor steps it takes to raise a carriage by 1 mm
#define STEPS_MM_EXT 30.000*16      // Number of stepper motor steps it takes to extrude 1 mm of filament

//Movement rates:
#define MAX_ACCEL_MM_SEC2 900.000   // Maximum cartesian acceleration of end effector in mm / s^2
#define MAX_MOVE_RATE_MM_SEC 120    // Maximum cartesian verlocity of end effector, in mm/s
#define HOME_RATE_MM_SEC 10         // Speed at which to home the endstops, in mm/s
#define MAX_EXT_RATE_MM_SEC 150     // Maximum rate at which filament should ever be extruded, in mm of filament / s


//Resistor-Capacitor thermistor read settings (see drivers/rcthermistor.h):
#define THERM_RA 665
#define THERM_CAP_PICO  2200000
#define VCC_mV 3300
#define THERM_IN_THRESH_mV 1600
#define THERM_T0 25
#define THERM_R0 100000
#define THERM_BETA 3950

//Pin Definitions:
//Note: these are all physical pin numbers, as opposed to logical pin numbers.
//  We don't use any of the pins' special features, 
//    so you should be able to choose whichever pins you find most convenient.
//  Please note, though, that you must invoke `make MAX_RPI_PIN_ID=NN' if you use a *logical* pin that maps to something > 31
//    This is not applicable to the RPI model A, B, A+ or B+, since all usable GPIOs are < 32, 
//    but it may be applicable to future versions
//  P1_01 is the top-left of the GPIO headers,
//  P1_02 is the top-right,
//  then the next row is P1_03, P1_04, and so on.
//  More information can be found in drivers/rpi/mitpi.h
#define PIN_STEPPER_EN     mitpi::V2_GPIO_P1_16
#define PIN_ENDSTOP_A      mitpi::V2_GPIO_P1_13
#define PIN_ENDSTOP_B      mitpi::V2_GPIO_P5_03
#define PIN_ENDSTOP_C      mitpi::V2_GPIO_P1_15
#define PIN_THERMISTOR     mitpi::V2_GPIO_P1_18
#define PIN_FAN            mitpi::V2_GPIO_P1_08
#define PIN_HOTEND         mitpi::V2_GPIO_P1_10

#define PIN_STEPPER_A_STEP mitpi::V2_GPIO_P1_22
#define PIN_STEPPER_A_DIR  mitpi::V2_GPIO_P1_23
#define PIN_STEPPER_B_STEP mitpi::V2_GPIO_P1_19
#define PIN_STEPPER_B_DIR  mitpi::V2_GPIO_P1_21
#define PIN_STEPPER_C_STEP mitpi::V2_GPIO_P1_24
#define PIN_STEPPER_C_DIR  mitpi::V2_GPIO_P1_26
#define PIN_STEPPER_E_STEP mitpi::V2_GPIO_P1_03
#define PIN_STEPPER_E_DIR  mitpi::V2_GPIO_P1_05

//PID thermistor->hotend feedback settings
//  We need to take the current temperature and use that to drive how much power we are sending to the hotend.
//  Note especially that a thermistor takes a few seconds to adjust, so there is some latency in readings.
//  The feedback algorithm is explained in pid.h and http://en.wikipedia.org/wiki/PID_controller
#define HOTEND_PID_P 18000
#define HOTEND_PID_I 250
#define HOTEND_PID_D 1000


/* Calibrating:
  as y leaves 0 to +side, z increases (should stay level)
    Even more so as it goes to -side.
  as x becomes +size, z increases
  as x becomes -size, z increases
  This points to either R or L being off, but in what way?
    joint to edge of bed is ~43 mm. bed is 170mm, so R is 43 + 85 = 128mm
    L is 215mm as measured BUT math doesn't consider the existence of an effector (so L should be longer?)
  Note: increasing L increases convexity (/\)
  Note: decreasing L increases concavity (\/)
  Note: increasing R increases concavity (\/)
  Note: decreasing R increases convexity (/\)
  Note: decreasing R decreases actual displacement (eg X100 becomes only 90mm from center)
  at 121, 222, 60mm in x dir is really 68mm.
*/

namespace machines {
namespace rpi {

using namespace drv; //for all the drivers
using namespace drv::rpi; //for RpiIoPin, etc.

class KosselPi : public Machine {
    private:
        //define one pin to enable/disable ALL steppers.
        //  This pin should be connected directly to the EN pin of each stepper motor driver chip in use.
        //  it defaults to HIGH=disabled, LOW=enabled.
        typedef InvertedPin<RpiIoPin<PIN_STEPPER_EN, IoHigh> > _StepperEn;
        
        //define the endstops:
        //  These endstops are wired as a switch, with one end tied to the 3.3V supply and the other 
        //    connected to a resistor of 1k-50k ohms, and then fed to the input. 
        //    The endstop is interpreted as triggered when the switch is OPEN 
        //    (so if you have a 3-pin endstop, use the common pin and the open pin)
        //  The resistor is technically optional, but without it, you run the risk of
        //    shorting Vcc to ground if the pin EVER gets misconfigured as an output 
        //    (this would destroy the pin & possibly damage the Pi).
        typedef Endstop<InvertedPin<RpiIoPin<PIN_ENDSTOP_A, IoLow, mitpi::GPIOPULL_DOWN> > > _EndstopA;
        typedef Endstop<InvertedPin<RpiIoPin<PIN_ENDSTOP_B, IoLow, mitpi::GPIOPULL_DOWN> > > _EndstopB;
        typedef Endstop<InvertedPin<RpiIoPin<PIN_ENDSTOP_C, IoLow, mitpi::GPIOPULL_DOWN> > > _EndstopC;
        
        //define the thermistor:
        //  The Raspberry Pi lacks an analog to digital converter, so we use a resistor-capacitor circuit to 
        //    measure the time it takes a fully charged capacitor of known capacitance to discharge through the thermistor.
        //  We have a single pin here, and we configure it as output(HIGH) to charge the capacitor, 
        //    and then configure it as input and measure the time it takes for the logic level to drop to 
        //    low while the capacitor discharges through a parallel connection to ground (through Ra).
        typedef RCThermistor<RpiIoPin<PIN_THERMISTOR>, THERM_RA, THERM_CAP_PICO, VCC_mV, THERM_IN_THRESH_mV, THERM_T0, THERM_R0, THERM_BETA> _Thermistor;
        
        //define the fan:
        //  The fan is controlled in a PWM way - set HIGH for full power, set LOW for off, 
        //    toggle rapidly for something in-between.
        //  One should not attempt to directly drive a fan off of the Pi's GPIOs.
        //  Instead, obtain a transistor, wire base to 12V, collector to the GPIO, and emitter should be
        //    connected through the fan and into ground.
        typedef Fan<RpiIoPin<PIN_FAN, IoLow> > _Fan;
        
        //define the hotend:
        //  The hotend is controlled in precisely the same manner as the fan.
        //  Please note that this is currently set to inverted mode, so a logic-level HIGH should 
        //    result in 0 power delivered to the hotend.
        typedef InvertedPin<RpiIoPin<PIN_HOTEND, IoHigh> > _HotendOut;
        
        //Expose the logic used to control the stepper motors:
        //Here we just have 1 stepper motor for each axis and another for the extruder:
        typedef std::tuple<LinearDeltaStepper<DELTA_AXIS_A, _EndstopA>, 
                           LinearDeltaStepper<DELTA_AXIS_B, _EndstopB>, 
                           LinearDeltaStepper<DELTA_AXIS_C, _EndstopC>, 
                           LinearStepper<CARTESIAN_AXIS_E> > _AxisStepperTypes;
        typedef AxisStepper::GetHomeStepperTypes<_AxisStepperTypes>::HomeStepperTypes _HomeStepperTypes;
        typedef AxisStepper::GetArcStepperTypes<_AxisStepperTypes>::ArcStepperTypes _ArcStepperTypes;
        
        //Gather all the I/O controlled devices we defined above:
        //  Additionally, define the actual stepper motor drivers and tie the thermistor to 
        //    the hotend as a feedback source.
        typedef std::tuple<
            A4988<RpiIoPin<PIN_STEPPER_A_STEP>, RpiIoPin<PIN_STEPPER_A_DIR>, _StepperEn>, //A tower
            A4988<RpiIoPin<PIN_STEPPER_B_STEP>, RpiIoPin<PIN_STEPPER_B_DIR>, _StepperEn>, //B tower
            A4988<RpiIoPin<PIN_STEPPER_C_STEP>, RpiIoPin<PIN_STEPPER_C_DIR>, _StepperEn>, //C tower
            A4988<RpiIoPin<PIN_STEPPER_E_STEP>, RpiIoPin<PIN_STEPPER_E_DIR>, _StepperEn>, //E coord
            _Fan,
            TempControl<drv::HotendType, 5, _HotendOut, _Thermistor, PID<HOTEND_PID_P, HOTEND_PID_I, HOTEND_PID_D>, LowPassFilter>
            > _IODriverTypes;
    public:
        //getXXX defines wrappers for all the above types. 
        //  Note that these should serve more as "factory" methods - creating objects - rather than as accessors.
        
        inline _IODriverTypes getIoDrivers() const {
            return _IODriverTypes(
                A4988<RpiIoPin<PIN_STEPPER_A_STEP>, RpiIoPin<PIN_STEPPER_A_DIR>, _StepperEn>(),
                A4988<RpiIoPin<PIN_STEPPER_B_STEP>, RpiIoPin<PIN_STEPPER_B_DIR>, _StepperEn>(),
                A4988<RpiIoPin<PIN_STEPPER_C_STEP>, RpiIoPin<PIN_STEPPER_C_DIR>, _StepperEn>(),
                A4988<RpiIoPin<PIN_STEPPER_E_STEP>, RpiIoPin<PIN_STEPPER_E_DIR>, _StepperEn>(),
                _Fan(),
                TempControl<drv::HotendType, 5, _HotendOut, _Thermistor, PID<HOTEND_PID_P, HOTEND_PID_I, HOTEND_PID_D>, LowPassFilter>(
                    _HotendOut(), _Thermistor(), PID<HOTEND_PID_P, HOTEND_PID_I, HOTEND_PID_D>(), LowPassFilter(3000)));
        }

        //Define the acceleration method to use. This uses a constant acceleration (resulting in linear velocity).
        inline ConstantAcceleration getAccelerationProfile() const {
            return ConstantAcceleration(MAX_ACCEL_MM_SEC2);
        }
        
        //Define the coordinate system:
        //  We are using a LinearDelta coordinate system, where vertically-moving carriages are
        //    attached to an end effector via fixed-length, rotatable rods.
        inline LinearDeltaCoordMap<> getCoordMap() const {
            //the Matrix3x3 defines the level of the bed:
            //  This is a matrix such that M * {x,y,z} should transform desired coordinates into a 
            //    bed-level-compensated equivalent.
            //  Usually, this is just a rotation matrix.
            return LinearDeltaCoordMap<>(R_MM, L_MM, H_MM, BUILDRAD_MM, STEPS_MM, STEPS_MM_EXT, Matrix3x3(
                0.999975003, 0.000005356, -0.007070522, 
                0.000005356, 0.999998852, 0.001515111, 
                0.007070522, -0.001515111, 0.999973855));
        }
        inline _AxisStepperTypes getAxisSteppers() const {
            return _AxisStepperTypes();
        }
        inline _HomeStepperTypes getHomeSteppers() const {
            return _HomeStepperTypes();
        }
        inline _ArcStepperTypes getArcSteppers() const {
            return _ArcStepperTypes();
        }

        //Expose default and maximum velocities:
        inline float defaultMoveRate() const { //in mm/sec
            return MAX_MOVE_RATE_MM_SEC;
        }
        //currently have to be satisfied with mins/maxes - can't achieve more without muddying the interface.
        inline float maxRetractRate() const { //in mm/sec
            return MAX_EXT_RATE_MM_SEC;
        }
        inline float maxExtrudeRate() const { //in mm/sec
            return MAX_EXT_RATE_MM_SEC;
        }
        inline float clampMoveRate(float inp) const {
            return std::min(inp, defaultMoveRate());
        }
        inline float clampHomeRate(float inp) const {
            (void)inp; //unused argument
            return HOME_RATE_MM_SEC;
        }
};

}
}

#endif
