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
 * This is a minimal Machine that will respond to gcode commands 
 *  and will simulate motion in cartesian space, 
 *  but is configured to not actually interact with any GPIOs
 *
 * Copy this file to src/machines/<platform>/cartesian.h where platform is e.g. "rpi",
 *   edit the pin definitions (reference another machine under your platform directory if possible),
 *   and then compile with `make MACHINE=machines/<platform>/cartesian.h`
 */

#ifndef MACHINES_GENERIC_CARTESIAN_H
#define MACHINES_GENERIC_CARTESIAN_H

#include <tuple>
#include "motion/constantacceleration.h"
#include "motion/linearcoordmap.h"
#include "machines/machine.h"
#include "common/matrix.h"
#include "iodrivers/endstop.h"
#include "iodrivers/a4988.h"
#include "iodrivers/fan.h"
#include "iodrivers/tempcontrol.h"
#include "iodrivers/rcthermistor2pin.h"
#include "pid.h"
#include "common/filters/lowpassfilter.h"
#include "iodrivers/servo.h"


//All of the #defines between this point and the end of this file are ONLY used within this file,

//Build calibration settings:
#define STEPS_MM_X 6.265*8            // Number of stepper motor steps it takes to move 1 mm in x direction
#define STEPS_MM_Y 6.265*8            // Number of stepper motor steps it takes to move 1 mm in y direction
#define STEPS_MM_Z 6.265*8            // Number of stepper motor steps it takes to move 1 mm in z direction
#define STEPS_MM_EXT 30.000*16        // Number of stepper motor steps it takes to extrude 1 mm of filament


//Movement rates:
#define MAX_ACCEL_MM_SEC2 900.000   // Maximum cartesian acceleration of end effector in mm / s^2
#define MAX_MOVE_RATE_MM_SEC 120    // Maximum cartesian verlocity of end effector, in mm/s
#define HOME_RATE_MM_SEC 10         // Speed at which to home the endstops, in mm/s
#define MAX_EXT_RATE_MM_SEC 150     // Maximum rate at which filament should ever be extruded, in mm of filament / s


//Pin Definitions:
//  replace the -1 placeholders with your actual pins.
//  to see how to do this, reference e.g. machines/rpi/kosselrampsfd.h
#define PIN_ENDSTOP_X             -1   
#define PIN_ENDSTOP_Y             -1
#define PIN_ENDSTOP_Z             -1
#define PIN_ENDSTOP_INVERSIONS    NO_INVERSIONS           //if not inverted, endstops are HIGH when active

//Refer to the RcThermistor2Pin documentation.
//One pin is used to discharge the capacitor through the thermistor (variable resistance)
//  This pin should be connected through the thermistor to P301-2
#define PIN_THERMISTOR            -1
//One pin (fixed resistance) is used for charging the capacitor.
//  This pin should be connected through a ~1kohm resistor to THERM0 (AD0)
#define PIN_THERMISTOR_CHARGE     -1
#define PIN_FAN                   -1    
#define PIN_FAN_INVERSIONS        INVERT_WRITES
#define PIN_FAN_DEFAULT_STATE     IO_DEFAULT_LOW
#define FAN_MIN_PWM_PERIOD        0.01                    

#define PIN_HOTEND                -1    
#define PIN_HOTEND_INVERSIONS     NO_INVERSIONS
#define HOTEND_MIN_PWM_PERIOD     0.01                    //MOSFETS have a limited switching frequency

#define PIN_STEPPER_X_EN          -1    
#define PIN_STEPPER_X_STEP        -1    
#define PIN_STEPPER_X_DIR         -1    

#define PIN_STEPPER_Y_EN          -1    
#define PIN_STEPPER_Y_STEP        -1    
#define PIN_STEPPER_Y_DIR         -1    

#define PIN_STEPPER_Z_EN          -1    
#define PIN_STEPPER_Z_STEP        -1    
#define PIN_STEPPER_Z_DIR         -1    

#define PIN_STEPPER_E_EN          -1    
#define PIN_STEPPER_E_STEP        -1    
#define PIN_STEPPER_E_DIR         -1    
#define PIN_STEPPER_EN_INVERSIONS INVERT_WRITES

//PID thermistor->hotend feedback settings
//  We need to take the current temperature and use that to drive how much power we are sending to the hotend.
//  Note especially that a thermistor takes a few seconds to adjust, so there is some latency in readings.
//  The feedback algorithm is explained in pid.h and http://en.wikipedia.org/wiki/PID_controller
#define HOTEND_PID_P 18.000
#define HOTEND_PID_I  0.250
#define HOTEND_PID_D  1.000

//Resistor-Capacitor thermistor read settings (see iodrivers/rcthermistor2pin.h):
#define THERM_C_FARADS            10.10e-6
#define THERM_V_TOGGLE_V          1.27
#define THERM_RCHARGE_OHMS        1000
#define THERM_RSERIES_OHMS          22
#define THERM_RUP_OHMS            4700
#define THERM_T0_C                25.0
#define THERM_R0_OHMS             100000
#define THERM_BETA                3950

#define VCC_V                     3.3  

namespace machines {
namespace generic {

using namespace iodrv; //for all the drivers
using namespace motion; //for Acceleration & such

class cartesian : public Machine {
    public:
        inline ConstantAcceleration getAccelerationProfile() const {
            return ConstantAcceleration(MAX_ACCEL_MM_SEC2);
        }
        inline LinearCoordMap<A4988, A4988, A4988, A4988> getCoordMap() const {
            return LinearCoordMap<A4988, A4988, A4988, A4988>(
                STEPS_MM_X, STEPS_MM_Y, STEPS_MM_Z, STEPS_MM_EXT, HOME_RATE_MM_SEC, 
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_X_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_X_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_X_EN)),
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_Y_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_Y_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_Y_EN)),
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_Z_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_Z_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_Z_EN)),
                A4988(IoPin(NO_INVERSIONS, PIN_STEPPER_E_STEP), 
                      IoPin(NO_INVERSIONS, PIN_STEPPER_E_DIR), 
                      IoPin(PIN_STEPPER_EN_INVERSIONS, PIN_STEPPER_E_EN)),
                Endstop(IoPin(PIN_ENDSTOP_INVERSIONS, PIN_ENDSTOP_X)),
                Endstop(IoPin(PIN_ENDSTOP_INVERSIONS, PIN_ENDSTOP_Y)),
                Endstop(IoPin(PIN_ENDSTOP_INVERSIONS, PIN_ENDSTOP_Z)),
                Matrix3x3( //bed level matrix. Coordinates are leveled by multiplying them with this matrix: P(leveled) = M*P(unleveled)
            1, 0, 0,
            0, 1, 0,
            0, 0, 1));
        }
        inline std::tuple<Fan, Servo, TempControl<RCThermistor2Pin, PID, LowPassFilter> > 
          getIoDrivers() const {
            return std::make_tuple(
                Fan(IoPin(PIN_FAN_INVERSIONS, PIN_FAN), PIN_FAN_DEFAULT_STATE, FAN_MIN_PWM_PERIOD),
                Servo(IoPin::null(), std::chrono::milliseconds(100), 
                    std::make_pair(std::chrono::milliseconds(1), std::chrono::milliseconds(2)),
                    std::make_pair(0.0f, 360.0f)),
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
                    LowPassFilter(3.000)
                ));
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
