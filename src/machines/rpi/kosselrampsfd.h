/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * Kossel Deltabot design that interfaces the Raspberry Pi to a RAMPS-FD (version 1 rev A) board:
 *   Geeetech RAMPS-FD files: http://www.geeetech.com/wiki/index.php/Ramps-FD#FILES
 *     Schematic: http://www.geeetech.com/wiki/images/8/83/RAMPS-FD-Schematic.pdf
 *   RAMPS-FD (RAMPS-For Due) board documentation: http://reprap.org/wiki/RAMPS-FD
 *   Board Schematic (note! This is for Rev 2): https://github.com/bobc/bobc_hardware/blob/master/RAMPS-FD/RAMPS-FD-Schematic.pdf
 *   Thread describing Ramps-fd v1 "fixes": http://forums.reprap.org/read.php?219,424146,435502
 *     don't appear to be any deal-breakers, just inconveniences
 *   Friendly Labeled Board Image: http://reprap.org/wiki/File:Ramps-FD_Diagram_-_Rev3.png
 *   Arduino MEGA/DUE pinout: http://arduino.cc/en/uploads/Reference/arduino_mega_ethernet_pins.png
 *     Note: the RAMPS-FD schematic uses the above pin labeling
 *     NOTE: the FD board has additional pins near RESET, 3V3, 5V, GND, VIN than listed in this photo!
 *       It would appear that "R3" arduinos have these extra pins?
 *   BETTER Arduino Due pinout: http://forum.arduino.cc/index.php?topic=132130.0
 *   Marlin may also be of reference for RAMPS-FD pinouts: https://github.com/bobc/Marlin/blob/Marlin_v1/Due/Marlin/pins.h#L170
 *   FD blog posts: http://andy-projects.blogspot.com/2014/02/ramps-fd-initial-setup.html
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
 *     All 6 fets produce either 12V or 0V across the connected device
 *   Note: Marlin uses HEATER_0_PIN = D9 -> maps to "Extruder 1" on RAMPS-FD, so "Extruder 1" must refer to the heating resistor in primary hotend
 * Power:
 *   Although Arduino Due uses 3.3V internally, it still exposes 5V to the 5V shield pin (taken from power adapter)
 *   P108, P107, HB_IN (3 blue adjacent screw terminals at the corner) are all 12-24V power inputs
 *   No screw terminal for logic (3.3 V) power - so it likely comes from the arduino header.
 *   Oddly, there are references to a 5V power source. The pin adjacent to 3.3V on arduino is labeled 5V and DUE_5V on the FD schematic.
 *     This 5V source is used for the "status" LED (pg 1) and then OPTIONALLY sent to V_LOGIC (pg 2).
 *      Also used in: 
         - SERVO1 through SERVO4 (pg 6), 
         - AUX3 - SPI (pg 6), 
         - ENDSTOPS (pg 7) - acts as a pullup, can be disabled by removing JP801
		     Endstop outputs are sent to an IO buffer chip, 4050_RMC, with Vlogic power supply. So likely still 3.3V compatible, 
		       so long as you avoid the unbuffered P802 (CONN_6X2) interface
		 - P801 - "CONN_6" (pg 7)
 *     There is a jumper - JP101 - to choose 3.3V or 5V logic levels. The note says: "VSEL - Select logic voltage. 1-2 = Autoselect by IOREF, 2-3 = 5V"
 *       Also a note: "On R3 compatible Arduinos, IOREF will supply 3.3V (DUE) or 5V (MEGA). If IOREF is not provided (non-R3) then it must be a 5V Arduino so set VSEL = 5V"
 *     There is also jumper JP201, "5V_SEL" that connects DUE_5V to FD_5V. FD_5V is also permanently connected to P219, "EXT_5V". This suggests that one can either use an external 5V supply and remove the jumper, or use DUE_5V and insert the jumper.
 *   It looks like all we need to do to be safe is:
 *     put 3.3V on the IOREF pin, OPTIONALLY 5V on the 5V pin, and set JP101 to "1-2 = Autoselect by IOREF"
 *     set JP102 to the appropriate power source; 1-2 = 24V, 2-3 = 12V.
 *         these are incorrectly labeled on the board such that "12" = 24V and "24" = 12V
 *     Remove JP1, for safety ("Supply 12V to Arduino in standalone operation")
 *     Set JP801 (endstop pull-ups)
 *
 *   Connecting motor power:
 *     put 12V to P107 (2nd blue terminal from corner). 
 *       Pin closest to board's corner is ground, according to https://raw.githubusercontent.com/bobc/bobc_hardware/master/RAMPS-FD/RAMPS-FD.png
 *   
 *   For thermistor reading:
 *     RPi only has digital inputs/outputs (can PWM outputs)
 *     But Ramps-FD gives us a few extra resistors, capacitors, diodes AND mosfets.
 *       Note: can create limited diode logic gates: http://en.wikipedia.org/wiki/Diode_logic#AND_logic_gate
 *     Previous RC reading designs here: https://github.com/Wallacoloo/printipi/issues/24
 *     RPi inputs have "hysteresis" - a pin may transition from reading LOW to HIGH at a 2v threshold, but HIGH -> LOW at a 1v threshold.
 *       This can be disabled, according to: http://www.mosaic-industries.com/embedded-systems/microcontroller-projects/raspberry-pi/gpio-pin-electrical-specifications
 *       But the procedure to do so is not described.
 *       We can also use one of the spare transistors on the FD board as a fixed comparator.
 *         Example (usting BJTs): http://pyevolve.sourceforge.net/wordpress/?p=2383
 *     One can create what is essentially a digitally-controlled resistor with a FET + capacitor: http://en.wikipedia.org/wiki/Switched_capacitor
 *     ADC Types: http://en.wikipedia.org/wiki/Analog-to-digital_converter#ADC_types
 *       Of note: delta-encoded ADC (same design as on the github issue: https://github.com/Wallacoloo/printipi/issues/24#issuecomment-58137624)
 *	     Of note: Successive Approximation ADC which is more resistance to oscillations around to signal than a delta-encoded ADC
 *     If we can convert an analog signal into a digital PWM signal, with duty cycle proportional to V,
 *       then we can sample the PWM signal with the RPi regulary and deduce the duty cycle via its average value.
 *       One such way to do this: compare Vin to some periodic analog signal
 *         Eg a sawtooth wave: http://electronics.stackexchange.com/questions/74138/voltage-controlled-pwm-generator
 *         Can produce a sinewave with 1 or 2 transistors (but might not achieve large voltage range):
 *           http://www.bowdenshobbycircuits.info/page8.htm#phase.gif
 *         Can also produce something CLOSE to a sinewave by outputting a square wave (with DMA) and low-pass filtering it with a cap
 */

#ifndef MACHINES_RPI_KOSSELRAMPSFD
#define MACHINES_RPI_KOSSELRAMPSFD

#include "machines/machine.h"
#include "pid.h"
#include "common/filters/lowpassfilter.h"
#include "common/matrix.h"
#include "motion/constantacceleration.h"
#include "iodrivers/a4988.h"
#include "motion/lineardeltacoordmap.h"
#include "iodrivers/rcthermistor.h"
#include "iodrivers/tempcontrol.h"
#include "iodrivers/fan.h"
#include "iodrivers/iopin.h"
#include "platforms/rpi/mitpi.h" //for pin numberings
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
//Note: supply the following power connections: 
//RPI:3.3V to FD:IOREF
//RPI:3.3V to FD:3.3V
//RPI:GND to FD:GND
//12V tp FD:P107 (MOT_IN) (2nd blue terminal from corner). Pin closest to corner is ground.
#define PIN_ENDSTOP_A      mitpi::V2_GPIO_P1_13 //maps to FD Shield D30 (X-MAX)
#define PIN_ENDSTOP_B      mitpi::V2_GPIO_P5_03 //maps to FD Shield D38 (Y-MAX)
#define PIN_ENDSTOP_C      mitpi::V2_GPIO_P1_15 //maps to FD Shield D34 (Z-MAX)
//#define PIN_THERMISTOR     mitpi::V2_GPIO_P1_18 //maps to FD Shield ?
#define PIN_FAN            mitpi::V2_GPIO_P1_08 //maps to FD Shield D12 (FET5)
#define PIN_HOTEND         mitpi::V2_GPIO_P1_10 //maps to FD Shield D9  (Extruder 1)

#define PIN_STEPPER_A_EN   mitpi::V2_GPIO_P5_04 //maps to FD Shield D48  (X_EN)
#define PIN_STEPPER_A_STEP mitpi::V2_GPIO_P1_22 //maps to FD Shield AD9  (X_STEP)
#define PIN_STEPPER_A_DIR  mitpi::V2_GPIO_P1_23 //maps to FD Sheild AD8  (X_DIR)

#define PIN_STEPPER_B_EN   mitpi::V2_GPIO_P5_05 //maps to FD Shield D46  (Y_EN)
#define PIN_STEPPER_B_STEP mitpi::V2_GPIO_P1_19 //maps to FD Shield AD11 (Y_STEP)
#define PIN_STEPPER_B_DIR  mitpi::V2_GPIO_P1_21 //maps to FD Shield AD10 (Y_DIR)

#define PIN_STEPPER_C_EN   mitpi::V2_GPIO_P5_06 //maps to FD Shield D44  (Z_EN)
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

using namespace iodrv; //for all the drivers
using namespace motion; //for Acceleration & such

class kosselrampsfd : public Machine {
    private:
        //define one pin to enable/disable ALL steppers.
        //  This pin should be connected directly to the EN pin of each stepper motor driver chip in use.
        //  it defaults to HIGH=disabled, LOW=enabled.
        
        //define the endstops:
        //  These endstops are wired as a switch, with one end tied to the 3.3V supply and the other 
        //    connected to a resistor of 1k-50k ohms, and then fed to the input. 
        //    The endstop is interpreted as triggered when the switch is OPEN 
        //    (so if you have a 3-pin endstop, use the common pin and the open pin)
        //  The resistor is technically optional, but without it, you run the risk of
        //    shorting Vcc to ground if the pin EVER gets misconfigured as an output 
        //    (this would destroy the pin & possibly damage the Pi).

        
        //define the thermistor:
        //  The Raspberry Pi lacks an analog to digital converter, so we use a resistor-capacitor circuit to 
        //    measure the time it takes a fully charged capacitor of known capacitance to discharge through the thermistor.
        //  We have a single pin here, and we configure it as output(HIGH) to charge the capacitor, 
        //    and then configure it as input and measure the time it takes for the logic level to drop to 
        //    low while the capacitor discharges through a parallel connection to ground (through Ra).
        
        //define the fan:
        //  The fan is controlled in a PWM way - set HIGH for full power, set LOW for off, 
        //    toggle rapidly for something in-between.
        //  One should not attempt to directly drive a fan off of the Pi's GPIOs.
        //  Instead, obtain a transistor, wire base to 12V, collector to the GPIO, and emitter should be
        //    connected through the fan and into ground.
        
        //define the hotend:
        //  The hotend is controlled in precisely the same manner as the fan.
        //  Please note that this is currently set to inverted mode, so a logic-level HIGH should 
        //    result in 0 power delivered to the hotend.
        
        
        
        //Gather all the I/O controlled devices:
        //  Additionally, define the actual stepper motor drivers and tie the thermistor to 
        //    the hotend as a feedback source.
        typedef std::tuple<
            A4988, //A tower
            A4988, //B tower
            A4988, //C tower
            A4988, //E coord. Note: the ordering of (A, B, C, E) in this tuple is strict (must be index 0, 1, 2 and then 3). But everything after that can be ordered any way
            Fan    //Hotend fan
            //TempControl<iodrv::HotendType, _HotendOut, _Thermistor, PID, LowPassFilter>
            > _IODriverTypes;
    public:
        //getXXX defines wrappers for all the above types. 
        //  Note that these should serve more as "factory" methods - creating objects - rather than as accessors.
        
        inline _IODriverTypes getIoDrivers() const {
            return std::make_tuple(
                A4988(IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_A_STEP), IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_A_DIR), IoPin(INVERT_WRITES, IoLow, PIN_STEPPER_A_EN)),
                A4988(IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_B_STEP), IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_B_DIR), IoPin(INVERT_WRITES, IoLow, PIN_STEPPER_B_EN)),
                A4988(IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_C_STEP), IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_C_DIR), IoPin(INVERT_WRITES, IoLow, PIN_STEPPER_C_EN)),
                A4988(IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_E_STEP), IoPin(NO_INVERSIONS, IoLow, PIN_STEPPER_E_DIR), IoPin(INVERT_WRITES, IoLow, PIN_STEPPER_E_EN)),
                Fan(IoPin(NO_INVERSIONS, IoLow, PIN_FAN)));
                //TempControl<iodrv::HotendType, _HotendOut, _Thermistor, PID, LowPassFilter>(
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
            return LinearDeltaCoordMap<>(R_MM, L_MM, H_MM, BUILDRAD_MM, STEPS_MM, STEPS_MM_EXT, 
                Endstop(IoPin(INVERT_READS, IoLow, PIN_ENDSTOP_A)),
                Endstop(IoPin(INVERT_READS, IoLow, PIN_ENDSTOP_B)),
                Endstop(IoPin(INVERT_READS, IoLow, PIN_ENDSTOP_C)),
                Matrix3x3(
                0.999975003, 0.000005356, -0.007070522, 
                0.000005356, 0.999998852, 0.001515111, 
                0.007070522, -0.001515111, 0.999973855));
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