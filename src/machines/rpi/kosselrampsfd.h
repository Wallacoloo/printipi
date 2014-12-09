/*
 * Kossel Deltabot design that interfaces the Raspberry Pi to a RAMPS-FD board:
 *   RAMPS-FD (RAMPS-For Due) board documentation: http://reprap.org/wiki/RAMPS-FD
 *   Board Schematic https://github.com/bobc/bobc_hardware/blob/master/RAMPS-FD/RAMPS-FD-Schematic.pdf
 *   Arduino MEGA/DUE pinout: http://arduino.cc/en/uploads/Reference/arduino_mega_ethernet_pins.png
 *     Note: the RAMPS-FD schematic uses the above pin labeling
 *   Marlin may also be of reference for RAMPS-FD pinouts: https://github.com/bobc/Marlin/blob/Marlin_v1/Due/Marlin/pins.h#L170
 *
 * Notes:
 *   RAMPS-FD (hereto dubbed 'FD') routes the stepper enable pins individually,
 *     so X-axis has X_EN,
 *        Y-axis has Y_EN, 
 *        ... Z_EN, E0_EN, E1_EN, E2_EN
 *   FD Shield has 6 endstops; X-MIN, X-MAX, ..., Z-MIN, Z-MAX
 *   All MOSFET outputs (heatbed, etc) are labeled as non-inverting
 *   Have 4 outputs for high-power things:
 *     1x heated bed,
 *     1x "Extruder 1" (This may actually be for the heating resistor within the nozzle)
 *     1x "Extruder 2 / Fan" (It appears this can be EITHER a second hotend or a general-purpose fan)
 *     1x "Extruder 3 / Fan"
 *     All of the above are functionally the same and use IRL88743PBF mosfets
 *   Then we get 2 extra FETs:
 *     FET5_BUF (D12) / FET6_BUF (D2)
 *     Same circuitry as the other 4 mosfets, but use part #DMN2075U
 *   Note: Marlin uses HEATER_0_PIN = D9 -> maps to "Extruder 1" on RAMPS-FD, so "Extruder 1" must refer to the heating resistor in primary hotend
 *
 */

#ifndef MACHINES_RPI_KOSSELRAMPSFD
#define MACHINES_RPI_KOSSELRAMPSFD

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

/* Calibrating:
  Note: increasing L increases convexity (/\)
  Note: decreasing L increases concavity (\/)
  Note: increasing R increases concavity (\/)
  Note: decreasing R increases convexity (/\)
  Note: decreasing R decreases actual displacement (eg X100 becomes only 90mm from center)
*/


//Movement rates:
#define MAX_ACCEL_MM_SEC2 900.000   // Maximum cartesian acceleration of end effector in mm / s^2
#define MAX_MOVE_RATE_MM_SEC 120    // Maximum cartesian verlocity of end effector, in mm/s
#define HOME_RATE_MM_SEC 10         // Speed at which to home the endstops, in mm/s
#define MAX_EXT_RATE_MM_SEC 150     // Maximum rate at which filament should ever be extruded, in mm of filament / s


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
#define PIN_ENDSTOP_A      mitpi::V2_GPIO_P1_13 //maps to FD Shield D30 (X-MAX)
#define PIN_ENDSTOP_B      mitpi::V2_GPIO_P5_03 //maps to FD Shield D38 (Y-MAX)
#define PIN_ENDSTOP_C      mitpi::V2_GPIO_P1_15 //maps to FD Shield D34 (Z-MAX)
//#define PIN_THERMISTOR     mitpi::V2_GPIO_P1_18 //maps to FD Shield ?
#define PIN_FAN            mitpi::V2_GPIO_P1_08 //maps to FD Shield D12 (FET5)
#define PIN_HOTEND         mitpi::V2_GPIO_P1_10 //maps to FD Shield D9  (Extruder 1)

#define PIN_STEPPER_A_EN   mitpi::V2_GPIO_P1_16 //maps to FD Shield D48  (X_EN)
#define PIN_STEPPER_A_STEP mitpi::V2_GPIO_P1_22 //maps to FD Shield AD9  (X_STEP)
#define PIN_STEPPER_A_DIR  mitpi::V2_GPIO_P1_23 //maps to FD Sheild AD8  (X_DIR)

#define PIN_STEPPER_B_EN   mitpi::V2_GPIO_P1_16 //maps to FD Shield D46  (Y_EN)
#define PIN_STEPPER_B_STEP mitpi::V2_GPIO_P1_19 //maps to FD Shield AD11 (Y_STEP)
#define PIN_STEPPER_B_DIR  mitpi::V2_GPIO_P1_21 //maps to FD Shield AD10 (Y_DIR)

#define PIN_STEPPER_C_EN   mitpi::V2_GPIO_P1_16 //maps to FD Shield D44  (Z_EN)
#define PIN_STEPPER_C_STEP mitpi::V2_GPIO_P1_24 //maps to FD Shield AD13 (Z_STEP)
#define PIN_STEPPER_C_DIR  mitpi::V2_GPIO_P1_26 //maps to FD Shield AD12 (Z_DIR)

#define PIN_STEPPER_E_EN   mitpi::V2_GPIO_P1_16 //maps to FD Shield D42  (E0_EN)
#define PIN_STEPPER_E_STEP mitpi::V2_GPIO_P1_03 //maps to FD Shield D36  (E0_STEP)
#define PIN_STEPPER_E_DIR  mitpi::V2_GPIO_P1_05 //maps to FD Shield D28  (E0_DIR)

//PID thermistor->hotend feedback settings
//  We need to take the current temperature and use that to drive how much power we are sending to the hotend.
//  Note especially that a thermistor takes a few seconds to adjust, so there is some latency in readings.
//  The feedback algorithm is explained in pid.h and http://en.wikipedia.org/wiki/PID_controller
#define HOTEND_PID_P 18.000
#define HOTEND_PID_I  0.250
#define HOTEND_PID_D  1.000

namespace machines {
namespace rpi {

using namespace drv; //for all the drivers
using namespace drv::rpi; //for RpiIoPin, etc.

class kosselrampsfd : public Machine {
    private:
        //define one pin to enable/disable ALL steppers.
        //  This pin should be connected directly to the EN pin of each stepper motor driver chip in use.
        //  it defaults to HIGH=disabled, LOW=enabled.
        typedef InvertedPin<RpiIoPin<PIN_STEPPER_A_EN, IoHigh> > _StepperAEn;
        typedef InvertedPin<RpiIoPin<PIN_STEPPER_B_EN, IoHigh> > _StepperBEn;
        typedef InvertedPin<RpiIoPin<PIN_STEPPER_C_EN, IoHigh> > _StepperCEn;
        typedef InvertedPin<RpiIoPin<PIN_STEPPER_E_EN, IoHigh> > _StepperEEn;

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
        //typedef RCThermistor<RpiIoPin<PIN_THERMISTOR> > _Thermistor;
        
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
        //typedef InvertedPin<RpiIoPin<PIN_HOTEND, IoHigh> > _HotendOut;
        
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
            A4988<RpiIoPin<PIN_STEPPER_A_STEP>, RpiIoPin<PIN_STEPPER_A_DIR>, _StepperAEn>, //A tower
            A4988<RpiIoPin<PIN_STEPPER_B_STEP>, RpiIoPin<PIN_STEPPER_B_DIR>, _StepperBEn>, //B tower
            A4988<RpiIoPin<PIN_STEPPER_C_STEP>, RpiIoPin<PIN_STEPPER_C_DIR>, _StepperCEn>, //C tower
            A4988<RpiIoPin<PIN_STEPPER_E_STEP>, RpiIoPin<PIN_STEPPER_E_DIR>, _StepperEEn>, //E coord
            _Fan
            //TempControl<drv::HotendType, _HotendOut, _Thermistor, PID, LowPassFilter>
            > _IODriverTypes;
    public:
        //getXXX defines wrappers for all the above types. 
        //  Note that these should serve more as "factory" methods - creating objects - rather than as accessors.
        
        inline _IODriverTypes getIoDrivers() const {
            return std::make_tuple(
                A4988<RpiIoPin<PIN_STEPPER_A_STEP>, RpiIoPin<PIN_STEPPER_A_DIR>, _StepperAEn>(),
                A4988<RpiIoPin<PIN_STEPPER_B_STEP>, RpiIoPin<PIN_STEPPER_B_DIR>, _StepperBEn>(),
                A4988<RpiIoPin<PIN_STEPPER_C_STEP>, RpiIoPin<PIN_STEPPER_C_DIR>, _StepperCEn>(),
                A4988<RpiIoPin<PIN_STEPPER_E_STEP>, RpiIoPin<PIN_STEPPER_E_DIR>, _StepperEEn>(),
                _Fan());
                //TempControl<drv::HotendType, _HotendOut, _Thermistor, PID, LowPassFilter>(
                //    _HotendOut(), _Thermistor(THERM_RA_OHMS, THERM_CAP_FARADS, VCC_V, THERM_IN_THRESH_V, THERM_T0_C, THERM_R0_OHMS, THERM_BETA), 
                //    PID(HOTEND_PID_P, HOTEND_PID_I, HOTEND_PID_D), LowPassFilter(3.000)));
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