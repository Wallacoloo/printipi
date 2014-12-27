#include "state.h"

#include <iostream>
#include <fstream> //for ifstream, ofstream
#include <thread>
#include <string>

#include "catch.hpp"
#include "compileflags.h"
#include "filesystem.h"
#include "gparse/com.h"

//MACHINE_PATH is calculated in the Makefile and then passed as a define through the make system (ie gcc -DMACHINEPATH='"path"')
#include MACHINE_PATH


//This test must be contained in a class so as to have access to special functions in the State.
struct TestClass {
    TestClass() {
        GIVEN("A State with Driver, Filesystem & Com interfaces") {
            //setup code:
            std::ofstream inputFile;
            //disable buffering before opening
            inputFile.rdbuf()->pubsetbuf(0, 0);
            inputFile.open("PRINTIPI_TEST_INPUT", std::fstream::out | std::fstream::trunc);
            std::ifstream outputFile;
            outputFile.rdbuf()->pubsetbuf(0, 0);
            //must open with the ::out flag to automatically create the file if it doesn't exist
            outputFile.open("PRINTIPI_TEST_OUTPUT", std::fstream::in | std::fstream::out | std::fstream::trunc);

            machines::MACHINE driver;
            FileSystem fs("/tmp/");
            gparse::Com com = gparse::Com("PRINTIPI_TEST_INPUT", "PRINTIPI_TEST_OUTPUT");
            State<machines::MACHINE> state(driver, fs, com, true);

            std::thread eventThread([&](){ 
                state.eventLoop(); 
            });

            //convenience function to read and wait for the next line from Printipi's output
            auto readLine = [&]() {
                std::string tempRead;
                do {
                    if (outputFile.eof()) {
                        outputFile.clear();
                    }
                } while (!std::getline(outputFile, tempRead));
                return tempRead;
            };

            //@cmd g-code command to send to printer (a newline character will be appended)
            //@expect expected response
            auto sendCommand = [&](const std::string &cmd, const std::string &expect) {
                INFO("Sending command: '" + cmd + "'");
                inputFile << cmd << '\n';
                INFO("It should be acknowledged with '" + expect + "'");
                REQUIRE(readLine() == expect);
            };

            //Verify that the position as reported by the motion planner is near (@x, @y, @z)
            auto verifyPosition = [&](float x, float y, float z) {
            	Vector4f actualPos = state.motionPlanner().actualCartesianPosition();
                INFO("Actual position: " + std::string(actualPos));
                REQUIRE(actualPos.xyz().distance(x, y, z) <= 4);
            };

            bool hasExited = false;
            auto exitOnce = [&]() {
                if (!hasExited) {
                    sendCommand("M0", "ok");
                    eventThread.join();
                    hasExited = true;
                }
            };

            //each WHEN/THEN case corresponds to a single test;
            //the above setup code and the teardown code further below are re-run for EVERY 'when' case.
            //This is also repeated recursively.

            //test homing
            WHEN("The machine is homed") {
                sendCommand("G28", "ok");
            }
            //test G1 movement
            WHEN("The machine is homed & moved to (40, -10, 50)") {
                sendCommand("G28", "ok");
                sendCommand("G1 X40 Y-10 Z50", "ok");
                //test G1 movement
                THEN("The actual position should be near (40, -10, 50)") {
	                exitOnce(); //force the G1 code to complete
	                verifyPosition(40, -10, 50);
            	}
            	//test successive G1 movements
            	WHEN("The machine is moved to another absolute position afterward, (-30, 20, 80) at F=3000") {
            		sendCommand("G1 X-30 Y20 Z80 F3000", "ok");
            		THEN("The actual position should be near (-30, 20, 80)") {
	            		exitOnce(); //force the G1 code to complete
	            		verifyPosition(-30, 20, 80);
            		}
            	}
            	//test G92 movement
            	WHEN("The machine is moved a RELATIVE amount (-70, 30, 30) at F=3000") {
            		sendCommand("G92", "ok");
            		sendCommand("G1 X-70 Y30 Z30 F3000", "ok");
            		THEN("The actual position should be near (-30, 20, 80)") {
	            		exitOnce(); //force the G1 code to complete
	            		verifyPosition(-30, 20, 80);
            		}
            	}
            }
            //test automatic homing
            WHEN("The machine is moved to (40, -10, 50) before being homed") {
            	sendCommand("G1 X40 Y-10 Z50", "ok");
                THEN("The actual position should be near (40, -10, 50)") {
	                exitOnce(); //force the G1 code to complete
	                verifyPosition(40, -10, 50);
            	}
            }
            //test automatic homing using G0
            WHEN("The machine is moved to (40, -10, 50) before being homed, using G0 command") {
            	sendCommand("G0 X40 Y-10 Z50", "ok");
                THEN("The actual position should be near (40, -10, 50)") {
	                exitOnce(); //force the G1 code to complete
	                verifyPosition(40, -10, 50);
            	}
            }

            //Teardown code:
            exitOnce();
        }
    }
};


SCENARIO("State will respond correctly to gcode commands", "[state]") {
    TestClass();
}