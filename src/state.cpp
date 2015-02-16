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

#include "state.h"

#include <iostream>
#include <fstream> //for ifstream, ofstream
#include <string>

#include "compileflags.h"
#include "platforms/auto/thisthreadsleep.h"
#include "common/logging.h"
#include "testhelper.h"

//MACHINE_PATH is calculated in the Makefile and then passed as a define through the make system (ie gcc -DMACHINEPATH='"path"')
#include MACHINE_PATH

SCENARIO("State will respond correctly to gcode commands", "[state]") {
    GIVEN("A State with Driver, Filesystem & Com interfaces") {
        //setup code:
        TestHelper<machines::MACHINE> helper(machines::MACHINE(), TESTHELPER_NO_PERSISTENT_ROOT_COM | TESTHELPER_ENTER_EVENT_LOOP);

        //each WHEN/THEN case corresponds to a single test;
        //the above setup code and the teardown code further below are re-run for EVERY 'when' case.
        //This is also repeated recursively.

        LOG("state.cpp: BEGIN TEST\n");
        //test homing
        helper.testHoming();
        //test G0/G1 linear movement
        helper.testLinearMovement();
        //test G2/G3 arc movement
        //helper.testArcMovement();

        //test automatic homing
        WHEN("The machine is moved to (30, -10, 15) before being homed") {
            helper.sendCommand("G1 X30 Y-10 Z15", "ok");
            THEN("The actual position should be near (30, -10, 15)") {
                helper.exitOnce(); //force the G1 code to complete
                helper.verifyPosition(30, -10, 15);
            }
        }
        //test linear movement using inch coordinates
        WHEN("The machine is moved to (-1, 2, 1) in inches") {
            //home machine
            helper.sendCommand("G28", "ok");
            //put into inches mode
            helper.sendCommand("G20", "ok");
            //move absolute
            helper.sendCommand("G1 X-1 Y2 Z1", "ok");
            THEN("The actual position (in mm) should be near (-1, 2, 1)*25.4") {
                helper.exitOnce(); //force the G1 code to complete
                helper.verifyPosition(-1*25.4, 2*25.4, 1*25.4);
            }
        }
        //test M18; let steppers move freely
        WHEN("The M18 command is sent to let the steppers move freely") {
            helper.sendCommand("M18", "ok");
            //"then the machine shouldn't crash"
        }
        //test gcode printing from file
        WHEN("Commands are read from a file with M32") {
            //home
            helper.sendCommand("G28", "ok");
            //"initialize" the SD card
            helper.sendCommand("M21", "ok");
            std::ofstream gfile("test-printipi-m32.gcode", std::fstream::out | std::fstream::trunc);
            //test newlines / whitespace
            gfile << "\n";
            gfile << " \t \n";
            //test comment & G90
            gfile << "G90 \t ; comment \n";
            gfile << "G1 X30 Y-10 Z15";
            AND_WHEN("The file is terminated with a newline") {
                //test ending the file WITHOUT a newline
                gfile << "\n" << std::flush;
                //load & run the file
                helper.sendCommand("M32 test-printipi-m32.gcode", "ok");
                THEN("The actual position should be near (30, -10, 15)") {
                    //note: Since running with TESTHELPER_NO_PERSISTENT_ROOT_COM, the M0 exit command from us is guaranteed to be processed
                    //  after the end of the gcode file, so no sleep is needed.
                    helper.exitOnce(); //force the G0 code to complete
                    helper.verifyPosition(30, -10, 15);
                }
            }
            AND_WHEN("The file does NOT end on an empty line") {
                //test ending the file WITHOUT a newline
                gfile << std::flush;
                //load & run the file
                helper.sendCommand("M32 test-printipi-m32.gcode", "ok");
                THEN("The actual position should be near (30, -10, 15)") {
                    //note: Since running with TESTHELPER_NO_PERSISTENT_ROOT_COM, the M0 exit command from us is guaranteed to be processed
                    //  after the end of the gcode file, so no sleep is needed.
                    helper.exitOnce(); //force the G0 code to complete
                    helper.verifyPosition(30, -10, 15);
                }
            }
            AND_WHEN("The file contains more commands after a M99 command") {
                gfile << "\n";
                gfile << "M99\n";
                gfile << "G1 X0 Y0 Z50\n" << std::flush;
                //load & run the file
                helper.sendCommand("M32 test-printipi-m32.gcode", "ok");
                THEN("No commands past M99 should be processed & the actual position should be near (30, -10, 15)") {
                    //note: Since running with TESTHELPER_NO_PERSISTENT_ROOT_COM, the M0 exit command from us is guaranteed to be processed
                    //  after the end of the gcode file, so no sleep is needed.
                    helper.exitOnce(); //force the G0 code to complete
                    helper.verifyPosition(30, -10, 15);
                }
            }
            remove("test-printipi-m32.gcode");
        }
        //test M84; stop idle hold (same as M18)
        WHEN("The M84 command is sent to stop the idle hold") {
            helper.sendCommand("M84", "ok");
            //"then the machine shouldn't crash"
        }
        WHEN("The M106 command is sent to activate fans") {
            helper.sendCommand("M106", "ok");
            WHEN("The M107 command is sent to disactivate fans") {
                helper.sendCommand("M107", "ok");
                //"then the machine shouldn't crash"
            }
        }
        WHEN("The M106 command is sent to activate fans at a specific PWM between 0.0-1.0") {
            helper.sendCommand("M106 S0.7", "ok");
            //"then the machine shouldn't crash"
        }
        WHEN("The M106 command is sent to activate fans at a specific PWM between 0-255") {
            helper.sendCommand("M106 S64", "ok");
            //"then the machine shouldn't crash", and S64 should be interpreted as 64/255 duty cycle.
        }
        WHEN("The M106 command is sent to activate a fan at a specific index") {
            helper.sendCommand("M106 P0", "ok");
            //"then the machine shouldn't crash"
        }
        WHEN("The M106 command is sent to activate a fan at an invalid index (-1)") {
            helper.sendCommand("M106 P-1", "ok");
            //"then the machine shouldn't crash"
        }
        WHEN("The M115 command is sent to get firmware info") {
            helper.sendCommand("M115", "ok");
        }
        WHEN("The M117 command is sent") {
            helper.sendCommand("M117 Hello, World!", "ok");
            //"then the machine shouldn't crash"
        }
         WHEN("The M117 command is sent") {
            helper.sendCommand("M119", "ok");
            //"then the machine shouldn't crash"
        }
        WHEN("The M280 command is sent with servo index=0") {
            helper.sendCommand("M280 P0 S40.5", "ok");
            //"then the machine shouldn't crash"
        }
        WHEN("The M280 command is sent with servo index=-1 (invalid)") {
            helper.sendCommand("M280 P-1 S40.5", "ok");
            //"then the machine shouldn't crash"
        }
        //Teardown code:
        // (Helper destructor)
    }
}