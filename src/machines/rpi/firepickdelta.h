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
         Endstop outputs are sent to an IO buffer chip, 4050_RMC (74HC4050), with Vlogic power supply. So likely still 3.3V compatible, 
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
 *       The procedure is listed here: http://www.scribd.com/doc/101830961/GPIO-Pads-Control2
 *         also: max current per pin is 16 mA (and total should not exceed 50 mA)
 *       We can also use one of the spare transistors on the FD board as a fixed comparator.
 *         Example (usting BJTs): http://pyevolve.sourceforge.net/wordpress/?p=2383
 *     One can create what is essentially a digitally-controlled resistor with a FET + capacitor: http://en.wikipedia.org/wiki/Switched_capacitor
 *     ADC Types: http://en.wikipedia.org/wiki/Analog-to-digital_converter#ADC_types
 *       Of note: delta-encoded ADC (same design as on the github issue: https://github.com/Wallacoloo/printipi/issues/24#issuecomment-58137624)
 *       Of note: Successive Approximation ADC which is more resistance to oscillations around to signal than a delta-encoded ADC
 *     If we can convert an analog signal into a digital PWM signal, with duty cycle proportional to V,
 *       then we can sample the PWM signal with the RPi regulary and deduce the duty cycle via its average value.
 *       One such way to do this: compare Vin to some periodic analog signal
 *         Eg a sawtooth wave: http://electronics.stackexchange.com/questions/74138/voltage-controlled-pwm-generator
 *         Can produce a sinewave with 1 or 2 transistors (but might not achieve large voltage range):
 *           http://www.bowdenshobbycircuits.info/page8.htm#phase.gif
 *         Can also produce something CLOSE to a sinewave by outputting a square wave (with DMA) and low-pass filtering it with a cap
 *
 *     Note the following thermistor table for T0=25*C, R0=100k, B=3950:
 *         R | T
 *    100000 |  25.00
 *     50000 |  41.46
 *     20000 |  66.23
 *     10000 |  87.72
 *      5000 | 112.12
 *      2000 | 149.93
 *      1000 | 183.86
 *       500 | 223.70
 *       200 | 288.43
 *       100 | 349.82
 *
 *     Can do classic RC-time measurements.
 *       Can also use a PWM signal w/ diode to gate the discharge (see below), so that we can keep discharge time more uniform across temperature.
 *       Major problem here is an unknown pin input threshold. Can use the buffer chips on the FD, but their specs aren't much better.
 *       Note: difficult to find a free diode in FD. I spot one for the heated bed, if the user isn't using that.
 *         the only free capacitors are at the thermistor bank, and those have some slightly undesirable connections.
 *         There is a cap from GND to V0 in LM7812 that's not connected to anything else (since we use 12V). Can we use that?
 *           Probably not. The 7812 is a voltage regulator. In this case, it's being used to bring 24V down to 12V,
 *             and is bypassed in the case that the input is already 12V. The capacitor connects output voltage to ground.
 *           It's possible that the output is floating when the input is < 14~15V (regulator has 2V drop-out),
 *             in which case we COULD use it. EDIT: Nope, the voltage is still 12V, even when tied to ground by 47k pull-down resistor.
 *         There aren't many free resistors. If we disable endstop pullups, then we have 3x 10k in parallel from unused endstops.
 *           Each stepper driver has a single 100k pull-down to ground.
 *           Fet 5 & 6 each have a 10k pull-down to ground (although it looks like their state is only undefined when the board loses power)
 *           ESTOP jumper has a 4.7k resistor connected to ground. 
 *             It's near the endstop section. The idea is that if you have a normally-closed switch, you can replace the jumper with that switch
 *             to use it as an emergency stop button.
 *             To make use of the 4k7 resistor, the pin closest to the endstops should be tied to ground, and the other pin has the resistor.
 *
 *
 *   Ramps-FD Thermistor port looks like:
 *
 *          V_LOGIC
 *             \
 *             / 4k7
 *             \
 *      22ohm  /
 *  o---/\/\/\-+------o
 *            _|_
 *            ___
 *             |
 *            GND
 *
 *      The 'o's are connection points.
 *      The intended use is that you connect your thermistor between the left o and GND to create a voltage divider,
 *        and read analog voltage at the right o.
 *
 *   Note on buffer circuitry:
 *     the 74HC4050 used to buffer endstops accepts 15V max input
 *     the other 74HC chips appear to only accept Vcc as max input, up to 7V
 *
 *   Possible thermistor circuits:
 *     1. Duty-cycle approximation
 *                                         
 *                Rt             
 *      Vcc ---/\/\/\/\---+-------\/\/\/\/\---------+ Node 2
 *                        |                        _|
 *                        |         (buf)         | |
 *                        +----------|>|---------||<.
 *                       _|_                      |_|
 *                       ___ C                      |
 *                        |                        GND
 *                       GND  
 *      In the above circuit, Rt is the thermistor, C is a capacitor, Rf is a feedback resistor,
 *        buffer is the 74HC125 chip on the FD board (if its input voltage > ~0.4V (?), then it outputs Vcc, else 0V)
 *        the mosfet serves as an inverting gate. when the buffer is HIGH, then Node 2 = Vgs (gate threshold, 0.4~1.0V), 
 *          else it's high-impedence
 *      The result is a feedback circuit, where the voltage across C is kept nearly constant (equal to the buffer's input threshold voltage).
 *        So we sample the output of the buffer (it serves as a comparator) many times and keep a rolling average of what we sample.
 *          This gives us a "duty cycle" - an idea of how often the buffer is active vs inactive.
 *          From this, we can calculate the average voltage at Node 2, and use that to solve for Rt.
 *      Possible problems:
 *        The feedback is limited by the mosfet switching frequency. I don't know how a fet behaves while switching, 
 *          so this notion of average voltage at Node 2 may be inaccurate.
 *        If this frequency is too high, capacitance at the input may become significant.
 *        What we might consider is to gate the feedback with square wave (generated via the Pi's DMA output).
 *          This would be done by placing a diode-based AND gate after the buffer, with the buffer as one input & square wave as the other.
 *          Or, we could cascade two fets
 *
 *     2. RC-time modification:
 *
 *                  Ra                   Rt      (diode)
 *       Vctrl ----/\/\/\/\----+------/\/\/\/\----->|--- Vdis
 *                            _|_              
 *                            ___ C             
 *                             |
 *                            GND
 *       
 *       In this circuit, a capacitor is charged, via setting Vctrl HIGH and Vdis to high-impedence.
 *       After waiting long enough for the capacitor to be nearly fully-charged,
 *         Vctrl is set to high-impedance (input mode), and Vdis is PWM'd at a constant value.
 *         This has the effect of discharging the capacitor through Rt,
 *         and we monitor Vctrl until it gets pulled low. At that instant, we know its approximate voltage and 
 *           can determine the time-constant of the time circuit, and from that, Rt.
 *       Note that the point of PWMing Vdis is to have more control over the time it takes to discharge the capacitor
 *         - when measuring a HIGH resistance (normally long discharge time), Vdis is set to 0.
 *         - when measuring a LOW resistance (normally short discharge time), Vdis is set to a higher PWM value.
 *
 *     3. RC-time w/ ramps-fd circuit:
 *             V_LOGIC
 *                 \
 *                 / 4k7
 *                 \
 *          22ohm  /         therm
 *      o---/\/\/\-+-----o-/\/\/\-o
 *      |         _|_    |        |
 *      # (buf)   ___    |        |
 *      |          |     |        |
 *     CHRG       GND   MEAS    DRAIN
 *    
 *      Here, we buffer an output from the Pi and set LOW to charge the cap. Then we switch it to high impedance (NOT POSSIBLE WITH THE BUFFER!)
 *      Next, DRAIN is pwm'd and the cap is discharged through a combination of the 4k7 and the thermistor.
 *        The PWM rate of DRAIN allows some control over the drainage time.
 *
 *     4. RC-time w/ ramps-fd circuit + pull resistors:
 *             V_LOGIC
 *                 \
 *                 / 4k7
 *                 \
 *          22ohm  /        3k3
 *      o---/\/\/\-+-----o-/\/\/\-o
 *      \         _|_             |
 *      / therm   ___             |
 *      \          |              |
 *    DRAIN       GND          CHRG/MEAS
 *    
 *      Modification to the previous circuit.
 *        pull CHRG low to bring the cap close to ground which DRAIN is disconnected.
 *        Simultaneously tie DRAIN high and make CHRG/MEAS high-impedance
 *        Use MEAS to measure how long it takes for cap to discharge.
 *      Here, we use 3x10k resistors in parallel, taken from the disabled X/Y/Z-MIN endstop pullups.
 *        This requires that JP801 is No-Connect, such that endstops have no pullups
 *        If we don't need ANY endstops, then we can get away with a full 6x10k resistors in parallel.
 *        Actually, we can wire all the endstops directly from GND to Pi input & use internal pull-up and then have the full 6x10k resistors available.
 *          Without the buffer, however, it's important to NOT expose the endstops to 5V (can perhaps disable 5V altogether)
 *      By pulling CHRG low, and disconnecting DRAIN, we can pull the capacitor down to 3.3 * (10000./3) / (10000./3 + 4700) = 1.37 V
 *        This is still in the undefined range of Pi inputs.
 *          Using the hex buffer chip doesn't help - 1.24V is the typical Vinput-low @ 3.3v (extrapolated).
 *          The quad/octa buffer chips have 1.43V typical Vinput-low @ 3.3v (extrapolated)
 *      So alternatively, we can set CHRG and DRAIN to Vcc to charge the cap, 
 *        and then pull both low and measure the dischage time. This still has a possibility to fail for large thermistor resistances, 
 *        but it should succeed for resistances < about 5k
 *      We could also use the 4k7 pull-down from the ESTOP jumper (near the endstop connections) to combat the 4k7 pull-up.
 *      DRAIN resistance | min cap voltage | max current
 *                  3k3 |            1.37 |  1.0 mA
 *                 1000 |            0.59 |  3.2 mA
 *                  680 |            0.43 |  4.7 mA
 *                  470 |            0.31 |  6.7 mA
 *                  330 |            0.23 |  9.4 mA
 *                  220 |            0.16 | 13.6 mA
 *                  180 |            0.14 | 16.3 mA (beyond pi ratings)
 *      Beyond ~280 C, the thermistor will be source too much current for the Pi.
 *        A solution is to place yet another resistor in series with the thermistor.
 *        Note: for PLA, we don't typically exceed 200 C, which is somewhere between 500-1000 ohms
 *          So we should be safe with no series resistor, and the Pi configured to source a max of 8 mA per pin.
 *      How to wire: 
 *                   P301 #2      -> thermistor -> Pi GPIO
 *                   THERM0 (AD0) -> 1k ohm     -> Pi GPIO
 *
 *
 *    5. RC-time w/ ramps-fd simple w/ diode:
 *                  V_LOGIC         V_LOGIC
 *                      \              |
 *                      / 4k7          |
 *                      \              |
 *      diode    22ohm  /       therm  |
 *     o-|<|-o---/\/\/\-+-----o-/\/\/\-o
 *     |               _|_    |        
 *     # (buf)         ___    |        
 *     |                |     |        
 *    CHRG             GND   MEAS    
 *
 *      In this version, the capacitor is charged by pulling CHRG low. 
 *        once charged, CHRG is pulled high, which puts the buffer high and the diode acts as high impedance.
 *      The capacitor is discharged through the parallel combination of 4k7 with therm. Measure the time it takes for MEAS to be pulled high.
 *      HOWEVER, this circuit requires a diode, and there aren't any diodes that can be used in this way on the board.
 *        ACTUALLY, there is a free diode if not using a heated bed (V_HEATBED is floating) in D8-Heatbed circuit
 *          edit: cannot use it without have a 1k8 resistor to 12V supply + led connected to the 22 ohm resistor.
 *
 *     6. RC-time w/ ramps-fd using fets:
 *                  V_LOGIC         V_LOGIC
 *                      \              |
 *                      / 4k7          |
 *                      \              |
 *               22ohm  /       therm  |
 *    P_FET6 o---/\/\/\-+-----o-/\/\/\-o
 *          _|         _|_    |        
 *         |           ___    |        
 *    o---||<.          |     |        
 *    |    |_|          |     |
 *    |      |          |     |
 *   CHRG   GND        GND   MEAS
 *
 *      Here we use a FET to control the charging. When CHRG is active, then that presents 0 V at P_FET6 to charge cap.
 *        When CHRG is inactive, then P_FET6 acts as high-impedance & doesn't interfere with circuit
 *      HOWEVER, there are no free fets on the board that can make this circuit 
 *        without having P_FET6 also have a connection to 12V through a 1k8 resistor + led
 *        BUT, there may be a possible workaround.
 *
 *     7. Duty-cycle Approximation w/ ramps-fd:
 *                                                       diode
 *                                                  .----->|--- V_POWER (12V) or V_HEATBED (NC)
 *                                                  |
 *                4k7         22ohm          buf2   |   LED    1k8
 *  V_LOGIC ---/\/\/\/\---+--\/\/\-o-------o-|<|-o--+---|<----\/\/\--- V_GATE (12v)
 *                        |                |       _|
 *                        |          buf1  |      | |      ^ LED is irrelevant
 *                        +-----o----|>|---}-----||<.         
 *                       _|_    \          |      |_|
 *                       ___ C  / Rtherm   |        |
 *                        |     \          |        |
 *                       GND   V_LOGIC   MEAS      GND
 *       This circuit is ACTUALLY constructable strictly with jumpers. o's indicate jumper patches.
 *         HOWEVER, it violates the maximum sourcing current of the buffer.
 *       First, the parallel combination of Rtherm | 4k7 will be supplying some amount of power to C.
 *       The buffers + fet create a feedback circuit that will keep the voltage across C constant (~1 diode drop).
 *         buf1 and buf2 will constantly be flipping.
 *         This means that buf2 will be alternately sourcing and draining power from the capacitor.
 *       When draining power, it's draining P=IV = V*V/R = D^2/22, where D is buf1's threshold voltage.
 *       When supplying power, it's supplying (V_LOGIC-D)^2/22.
 *       By sampling MEAS often, we can determine how much of the time buf2 is draining vs supplying, 
 *         and from that calculate the average amount of power being drained.
 *       The power being drained is equal to the power entering the capacitor through Rtherm | 4k7,
 *         so we can solve for Rtherm.
 *       DOWNSIDE is that this circuit attempts to switch the buffers / fet at a VERY high frequency, in which they might operate linearly and not digitally.
 *         The only way to really solve this is to use a Schmitt trigger (hysteresis).
 *         Rather than trying to implement in hardware, we could try to implement realtime feedback in software.
 *           given i = c*dv/dt, c=10e-6, then when FB circuit is active, if Rfb=22, then dv/dt = i/c = v/r/c = 3.3/22/10e-6 = 15000. 
 *             That is, after 10 us, C voltage will change by 0.15 V away from equilibrium, if not properly fedback.
 *           If we want to preserve the assumption that voltage across C is const, 0.1 V variation is probably the max we can accept.
 *           That requires a 7 us feedback frequency. Not doable in Linux userland, but perhaps doable on the GPU core.
 */

#ifndef MACHINES_RPI_FIREPICKDELTA
#define MACHINES_RPI_FIREPICKDELTA

#include <tuple>

#include "machines/machine.h"
#include "pid.h"
#include "common/filters/lowpassfilter.h"
#include "common/matrix.h"
#include "motion/constantacceleration.h"
#include "iodrivers/a4988.h"
#include "motion/angulardeltacoordmap.h"
#include "iodrivers/rcthermistor2pin.h"
#include "iodrivers/tempcontrol.h"
#include "iodrivers/fan.h"
#include "iodrivers/iopin.h"
#include "platforms/rpi/mitpi.h" //for pin numberings
#include "iodrivers/servo.h"





//*****************************************************************************************************************
//Firepick-Delta Settings


//Accounting for pulley reduction system 
#define BELT_THICKNESS 1.49 					// thickest part of belt
#define TOOTH_DEPTH 0.74 					// thinnest part of the belt
#define TOOTH_SPACING 2.0 					// 2mm for GT2 belts

// recalculate circumference to take into account tooth thickness, as this adds to the radius of the pulley
#define BIG_PULLEY_CIRCUM (BIG_PULLEY_TEETH*TOOTH_SPACING + (BELT_THICKNESS - TOOTH_DEPTH))*6.283185
#define SMALL_PULLEY_CIRCUM SMALL_PULLEY_TEETH * TOOTH_SPACING*6.283185

#define XYZ_FULL_STEPS_PER_ROTATION 200.0               	//1.8° steppers--> 200.0 | 0.9°steppers --> 400.0 
#define XYZ_MICROSTEPS 32.0					//DRV8825 set to 1/32 stepping
#define SMALL_PULLEY_TEETH 16.0					//GT2 pulley
#define BIG_PULLEY_TEETH 150.0					//3D002 Pulley - FirePickDelta
//#define PULLEY_REDUCTION BIG_PULLEY_CIRCUM/SMALL_PULLEY_CIRCUM
#define PULLEY_REDUCTION BIG_PULLEY_TEETH/SMALL_PULLEY_TEETH
#define XYZ_STEPS (XYZ_FULL_STEPS_PER_ROTATION*XYZ_MICROSTEPS*PULLEY_REDUCTION)/360.0
// probably not needed but defined for now 
#define STEPS_XYZ 360.0/(XYZ_FULL_STEPS_PER_ROTATION*XYZ_MICROSTEPS*PULLEY_REDUCTION)

//For Delta configuration: Units are degrees! That is, steps per degree

#define DEFAULT_AXIS_STEPS_PER_UNIT {XYZ_STEPS, XYZ_STEPS, XYZ_STEPS, 8.8888}


//All of the #defines between this point and the end of this file are ONLY used within this file,

//Build calibration settings:
#define R_MM 111.000                // Distance from the center of the build plate to each delta tower, in mm
#define L_MM 221.000                // Length of the vertical rods that connect the carriages to the end effector, in mm
#define H_MM 467.450                // The vertical distance from your endstops to the build platform, in mm
//#define BUILDRAD_MM 150.000          // The radius of your printbed, in mm (used to ensure desired coordinates are valid) -----> REPLACED WITH DELTA_PRINTABLE_RADIUS
#define STEPS_MM XYZ_STEPS            // Number of stepper motor steps it takes to raise a carriage by 1 mm  --> SHOUDL BE CHANGED TO STEPS_XYZ - HARD HACK4 NOW
#define STEPS_MM_EXT 30.000*16      // Number of stepper motor steps it takes to extrude 1 mm of filament

//Movement rates:
#define MAX_ACCEL_MM_SEC2 900.000   // Maximum cartesian acceleration of end effector in mm / s^2
#define MAX_MOVE_RATE_MM_SEC 120    // Maximum cartesian verlocity of end effector, in mm/s
#define HOME_RATE_MM_SEC 10         // Speed at which to home the endstops, in mm/s
#define MAX_EXT_RATE_MM_SEC 150     // Maximum rate at which filament should ever be extruded, in mm of filament / s



//#define DEFAULT_MAX_FEEDRATE {5000, 5000, 5000, 10000} // (mm/sec)
//#define DEFAULT_MAX_ACCELERATION {3000, 3000, 3000, 10000} // X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
//#define DEFAULT_ACCELERATION 3000//10000 // X, Y, Z and E max acceleration in mm/s^2 for printing moves
//#define DEFAULT_RETRACT_ACCELERATION 3000 // X, Y, Z and E max acceleration in mm/s^2 for retracts

#define DELTA
// Make delta curves from many straight lines (linear interpolation).
// This is a trade-off between visible corners (not enough segments)
// and processor overload (too many expensive sqrt calls).
#define DELTA_SEGMENTS_PER_SECOND 50

// Delta Dimensions
#define DELTA_E 131.636 // End effector length
#define DELTA_F 190.526 // Base length
#define DELTA_RE 270.000 // Carbon rod length
#define DELTA_RF 90.000 // Servo horn length

//#define DELTA_Z_OFFSET 293.000 // Distance from delta 8mm rod/pulley to table/bed.
//NOTE: For OpenPnP, set the zero to be about 25mm above the bed...
#define DELTA_Z_OFFSET 268.000 // Distance from delta 8mm rod/pulley to table/bed. --> Set for openpnp config
#define DELTA_EE_OFFS 15.000 // Ball joint plane to bottom of end effector surface



//#define TOOL_OFFSET 40.000 // Distance between end effector ball joint plane and tip of tool (Z probe)
#define TOOL_OFFSET 30.500 // Distance between end effector ball joint plane and tip of tool (PnP)


#define Z_CALC_OFFSET ((DELTA_Z_OFFSET - TOOL_OFFSET - DELTA_EE_OFFS) * -1.0)
#define Z_HOME_ANGLE -67.200 // This is the angle where the arms hit the endstop sensor
#define Z_HOME_OFFS (((DELTA_Z_OFFSET - TOOL_OFFSET - DELTA_EE_OFFS) - 182.002) - 0.5)
// This is calculated from the above angle, after applying forward
// kinematics, and adding the Z calc offset to it.
// Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
#define DELTA_PRINTABLE_RADIUS 150.0






//*****************************************************************************************************************
//*****************************************************************************************************************




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
//RPI:3.3V to FD:5V (NOT A TYPO - the FD board has endstop-pullups to "5V" (which are then buffered to 3.3V)). 
//  to be extra safe, we want to avoid ANY 5V where possible, so we supply the FD "5V" source with an actual 3.3V.
//  if you actually need the 5V somewhere, it is safe to connect 5V -> 5V, so long as you're extra careful wiring the rest.
//RPI:GND to FD:GND
//12V to FD:P108 (V_POWER; mosfet power suuply) (1st blue terminal from corner). Pin closest to corner is ground.
//12V tp FD:P107 (MOT_IN) (2nd blue terminal from corner). Pin closest to corner is ground.


//define the endstops:
//  These endstops are wired as a switch, with one end tied to the 3.3V supply and the other 
//    connected to a resistor of 1k-50k ohms, and then fed to the input. 
//    The endstop is interpreted as triggered when the switch is OPEN 
//    (so if you have a 3-pin endstop, use the common pin and the open pin)
//  The resistor is technically optional, but without it, you run the risk of
//    shorting Vcc to ground if the pin EVER gets misconfigured as an output 
//    (this would destroy the pin & possibly damage the Pi).
#define PIN_ENDSTOP_A             mitpi::V2_GPIO_P1_11             //GPIO# 17 on header
#define PIN_ENDSTOP_B             mitpi::V2_GPIO_P1_12             //GPIO# 18 on header
#define PIN_ENDSTOP_C             mitpi::V2_GPIO_P1_15             //GPIO# 22 on header
//endstops expects a HIGH input when the endstop is reached
#define PIN_ENDSTOP_INVERSIONS    NO_INVERSIONS
#define PIN_ENDSTOP_PULL          mitpi::GPIOPULL_UP

//define the thermistor:
//  The Raspberry Pi lacks an analog to digital converter, so we use a resistor-capacitor circuit to 
//    measure the time it takes a fully charged capacitor of known capacitance to discharge through the thermistor.
//  We have a single pin here, and we configure it as output(HIGH) to charge the capacitor, 
//    and then configure it as input and measure the time it takes for the logic level to drop to 
//    low while the capacitor discharges through a parallel connection to ground (through Ra).
//#define PIN_THERMISTOR         mitpi::V2_GPIO_P1_18    //maps to FD Shield ?

//define the fan:
//  The fan is controlled in a PWM way - set HIGH for full power, set LOW for off, 
//    toggle rapidly for something in-between.
//  One should not attempt to directly drive a fan off of the Pi's GPIOs.
//  Instead, obtain a transistor, wire base to 12V, collector to the GPIO, and emitter should be
//    connected through the fan and into ground.
#define PIN_FAN                   mitpi::V2_GPIO_P1_03           //maps to FD Shield D10 (Extruder 2 / Fan)
#define PIN_FAN_INVERSIONS        INVERT_WRITES
#define PIN_FAN_DEFAULT_STATE     IO_DEFAULT_LOW
//during RPi boot, fan can be set to be either on or off by using internal pull resistors
#define PIN_FAN_PULL              mitpi::GPIOPULL_UP
#define FAN_PWM_MULTIPLIER        0.60                           //weight the fan's PWM value to be XXX * (whatever value received by M106)
#define FAN_IDEAL_PWM_PERIOD        std::chrono::milliseconds(10)  //MOSFETS have a limited switching frequency             

#define PIN_STEPPER_A_EN          mitpi::V2_GPIO_P1_16		//GPIO# 23 on header  (x_EN)
#define PIN_STEPPER_A_STEP        mitpi::V2_GPIO_P1_07		//GPIO# 4  on header  (X_STEP)
#define PIN_STEPPER_A_DIR         mitpi::V2_GPIO_P1_26       	//GPIO# 7  on header  (X_DIR)

#define PIN_STEPPER_B_EN          mitpi::V2_GPIO_P1_18		//GPIO# 24 on header  (Y_EN)
#define PIN_STEPPER_B_STEP        mitpi::V2_GPIO_P1_24		//GPIO# 8  on header (Y_STEP)
#define PIN_STEPPER_B_DIR         mitpi::V2_GPIO_P1_21		//GPIO# 9  on header (Y_DIR)

#define PIN_STEPPER_C_EN          mitpi::V2_GPIO_P1_22		//GPIO# 25 on header  (Z_EN)
#define PIN_STEPPER_C_STEP        mitpi::V2_GPIO_P1_19		//GPIO# 10 on header (Z_STEP)
#define PIN_STEPPER_C_DIR         mitpi::V2_GPIO_P1_23		//GPIO# 11 on header (Z_DIR)

#define PIN_STEPPER_E_EN          mitpi::V2_GPIO_P5_03           //maps to FD Shield D42  (E0_EN)
#define PIN_STEPPER_E_STEP        mitpi::V2_GPIO_P5_04           //maps to FD Shield D36  (E0_STEP)
#define PIN_STEPPER_E_DIR         mitpi::V2_GPIO_P5_05           //maps to FD Shield D28  (E0_DIR)
//A4988 EN pin is logically inverted (writing HIGH to the /EN pin disables the stepper motor)
#define PIN_STEPPER_EN_INVERSIONS INVERT_WRITES
//Set the pin to pull HIGH so that the stepper is disabled during RPi boot
#define PIN_STEPPER_EN_PULL       mitpi::GPIOPULL_UP

//define the hotend:
//  The hotend is controlled in the same PWM manner as the fan
#define PIN_HOTEND                mitpi::V2_GPIO_P5_06           //maps to FD Shield D9  (Extruder 1)
#define PIN_HOTEND_INVERSIONS     INVERT_WRITES                  //Ramps-FD mosfets are inverted
//hotend state during boot, if hardware pull resistors weren't present.
//NOTE: if you have a hotend that is active LOW, then you want the pull resistor to pull HIGH!
#define PIN_HOTEND_PULL           mitpi::GPIOPULL_DOWN
//MOSFETS have a limited switching frequency, which can be accounted for with a minimum PWM_PERIOD
#define HOTEND_IDEAL_PWM_PERIOD   std::chrono::milliseconds(10)

//Refer to the RcThermistor2Pin documentation.
//One pin is used to discharge the capacitor through the thermistor (variable resistance)
//  This pin should be connected through the thermistor to P301-2
#define PIN_THERMISTOR            mitpi::V2_GPIO_P1_13
//One pin (fixed resistance) is used for charging the capacitor.
//  This pin should be connected through a ~1kohm resistor to THERM0 (AD0)
#define PIN_THERMISTOR_CHARGE     mitpi::V2_GPIO_P1_08
#define THERM_C_FARADS            10.10e-6
//With hysteresis disabled, at what input voltage does a pin (specifically, PIN_THERMISTOR) transition states?
// This video suggests some results with hysteresis enabled, and we average them to get 1.27 V: https://www.youtube.com/watch?v=Wr49ia3oID4
// Note that this value will be automatically re-calibrated at startup by default.
#define THERM_V_TOGGLE_V          1.27
#define THERM_RCHARGE_OHMS        1000
#define THERM_RSERIES_OHMS          22
#define THERM_RUP_OHMS            4700
#define THERM_T0_C                25.0
#define THERM_R0_OHMS             100000
#define THERM_BETA                3950

#define VCC_V                     3.3  


//PID thermistor->hotend feedback settings
//  We need to take the current temperature and use that to drive how much power we are sending to the hotend.
//  Note especially that a thermistor takes a few seconds to adjust, so there is some latency in readings.
//  The feedback algorithm is explained in pid.h and http://en.wikipedia.org/wiki/PID_controller
#define HOTEND_PID_P  0.018000
#define HOTEND_PID_I  0.000250
#define HOTEND_PID_D  0.001000

namespace machines {
namespace rpi {

using namespace iodrv; //for all the drivers
using namespace motion; //for ConstantAcceleration & such

class firepickdelta : public Machine {
    public:
        //getXXX define wrappers for all the above types. 
        //  Note that these should serve more as "factory" methods - creating objects - rather than as accessors.
        
        //return a list of miscellaneous IoDrivers (Endstops & A4988 drivers are reachable via <getCoordMap>)
        inline std::tuple<Fan, TempControl<RCThermistor2Pin, PID, LowPassFilter> > getIoDrivers() const {
            return std::make_tuple(
                Fan(IoPin(PIN_FAN_INVERSIONS, PIN_FAN, PIN_FAN_PULL), PIN_FAN_DEFAULT_STATE, FAN_PWM_MULTIPLIER, FAN_IDEAL_PWM_PERIOD),
                TempControl<RCThermistor2Pin, PID, LowPassFilter>(
                    iodrv::HotendType,
                    IoPin(PIN_HOTEND_INVERSIONS, PIN_HOTEND), 
                    RCThermistor2Pin(
                        IoPin(NO_INVERSIONS, PIN_THERMISTOR),
                        IoPin(NO_INVERSIONS, PIN_THERMISTOR_CHARGE),
                        THERM_RCHARGE_OHMS, 
                        THERM_RSERIES_OHMS,
                        THERM_RUP_OHMS,
                        THERM_C_FARADS, 
                        VCC_V, 
                        THERM_V_TOGGLE_V, 
                        THERM_T0_C, 
                        THERM_R0_OHMS, 
                        THERM_BETA), 
                    PID(HOTEND_PID_P, HOTEND_PID_I, HOTEND_PID_D), 
                    LowPassFilter(3.000),
                    HOTEND_IDEAL_PWM_PERIOD
                )
            );
                //TempControl<_HotendOut, _Thermistor, PID, LowPassFilter>(
                //    iodrv::HotendType, _HotendOut(), _Thermistor(THERM_RA_OHMS, THERM_CAP_FARADS, VCC_V, THERM_IN_THRESH_V, THERM_T0_C, THERM_R0_OHMS, THERM_BETA), 
                //    PID(HOTEND_PID_P, HOTEND_PID_I, HOTEND_PID_D), LowPassFilter(3.000)));
        }

        //Define the acceleration method to use. This uses a constant acceleration (resulting in linear velocity).
        inline ConstantAcceleration getAccelerationProfile() const {
 		LOG("fpdelta XYZ_STEPS: %f\n", XYZ_STEPS);
            return ConstantAcceleration(MAX_ACCEL_MM_SEC2);
        }
        

        //Define the coordinate system:
        //  We are using a LinearDelta coordinate system, where vertically-moving carriages are
        //    attached to an end effector via fixed-length, rotatable rods.
        inline AngularDeltaCoordMap<A4988, A4988, A4988, A4988> getCoordMap() const {
            //the Matrix3x3 defines the level of the bed:
            //  This is a matrix such that M * {x,y,z} should transform desired coordinates into a 
            //    bed-level-compensated equivalent.
            //  Usually, this is just a rotation matrix.
            LOG("fpdelta XYZ_STEPS: %f\n", XYZ_STEPS);
            return AngularDeltaCoordMap<A4988, A4988, A4988, A4988>(
                DELTA_E, DELTA_F, DELTA_RE, DELTA_RF, DELTA_Z_OFFSET, DELTA_PRINTABLE_RADIUS, XYZ_STEPS, STEPS_MM_EXT, HOME_RATE_MM_SEC, Z_HOME_ANGLE,
                //A tower:
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_A_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_A_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_A_EN, PIN_STEPPER_EN_PULL)),
                //B tower:
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_B_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_B_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_B_EN, PIN_STEPPER_EN_PULL)),
                //C tower:
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_C_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_C_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_C_EN, PIN_STEPPER_EN_PULL)),
                //Extruder axis:
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_E_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_E_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_E_EN, PIN_STEPPER_EN_PULL)),
                //A, B, C axis upper endstops:
                Endstop(IoPin(PIN_ENDSTOP_INVERSIONS, PIN_ENDSTOP_A, PIN_ENDSTOP_PULL)),
                Endstop(IoPin(PIN_ENDSTOP_INVERSIONS, PIN_ENDSTOP_B, PIN_ENDSTOP_PULL)),
                Endstop(IoPin(PIN_ENDSTOP_INVERSIONS, PIN_ENDSTOP_C, PIN_ENDSTOP_PULL)),
                //bed-level matrix
                Matrix3x3(
                1.000000000, 0.000000000, 0.000000000, 
                0.000005356, 1.000000000, 0.000000000, 
                0.000000000, 0.000000000, 1.000000000));
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
};

}
}


#endif
