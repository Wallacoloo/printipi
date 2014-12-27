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

            auto sendCommand = [&](const std::string &cmd, const std::string &expect) {
                INFO("Sending command: '" + cmd + "'");
                inputFile << cmd << '\n';
                INFO("It should be acknowledged with '" + expect + "'");
                REQUIRE(readLine() == expect);
            };

            bool hasExited = false;
            auto exitOnce = [&]() {
                if (!hasExited) {
                    sendCommand("M0", "ok");
                    eventThread.join();
                    hasExited = true;
                }
            };

            //each WHEN case corresponds to a single test;
            //the above setup code and the teardown code further below are re-run for EVERY 'when' case.
            WHEN("The machine is homed") {
                sendCommand("G28", "ok");
            }
            WHEN("The machine is homed & moved to (40, -10, 50)") {
                sendCommand("G28", "ok");
                sendCommand("G1 X40 Y-10 Z50", "ok");
                exitOnce(); //force the G1 code to complete
                Vector4f actualPos = state.motionPlanner().actualCartesianPosition();
                INFO("Actual position: " + std::string(actualPos));
                REQUIRE(actualPos.xyz().distance(40, -10, 50) <= 4);
             }

            //Teardown code:
            exitOnce();
        }
    }
};


SCENARIO("State will respond correctly to gcode commands", "[state]") {
    TestClass();
}