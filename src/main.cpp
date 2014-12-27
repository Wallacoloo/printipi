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
 * Printipi/main.cpp
 *
 * main is the program's main entry point.
 * main handles command line arguments, and instantiates the serial communications, State, and machine-specific driver.
 * Driver type is passed into this compilation unit as a command-line argument (gcc -DMACHINE=...)
 *
 * Printipi discussions:
 *   http://forums.reprap.org/read.php?2,396157
 *   https://groups.google.com/forum/#!searchin/deltabot/wallacoloo|sort:relevance/deltabot/JQNpmnlYYUc/_6V6SYcOGMUJ
 *   http://youtube.com/watch?v=g4UD5MRas3E
 */


#define COMPILING_MAIN //used elsewhere to do one-time warnings, etc.

#include "compileflags.h"

//Includes for the CATCH testing framework
#include <iostream>
#include <fstream> //for ifstream, ofstream
#include <thread>
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#include <string>
#include <sys/mman.h> //for mlockall
#include "common/logging.h"

#include "gparse/com.h"
#include "state.h"
#include "argparse.h"
#include "filesystem.h"

//MACHINE_PATH is calculated in the Makefile and then passed as a define through the make system (ie gcc -DMACHINEPATH='"path"')
//To set the path, call make MACHINE_PATH=...
//or, call make MACHINE=<machine>, eg MACHINE=rpi::KosselPi (case-sensitive) and the path will be calculated from that (src/machines/rpi/kossel.h)
#include MACHINE_PATH

void printUsage(char* cmd) {
    //#ifndef NO_USAGE_INFO
    LOGE("usage: %s [input file=/dev/stdin] [output file=/dev/null] [--help] [--quiet] [--verbose]\n", cmd);
    LOGE("examples:\n");
    LOGE("  print a gcode file: %s file.gcode\n", cmd);
    LOGE("  mock serial port: %s /dev/tty3dpm /dev/tty3dps\n", cmd);
}

int main_(int argc, char** argv) {
    //if the program was compiled in test mode, then run the unit tests and exit
    #if DO_TESTS
        int result = Catch::Session().run(argc, argv);
        return result;
    #else
        std::string defaultSerialFile("/dev/stdin");
        std::string serialFileName;
        std::string outFile = gparse::Com::NULL_FILE_STR;
        SchedulerBase::configureExitHandlers(); //useful to do this first-thing for catching debug info.
        
        if (argparse::cmdOptionExists(argv, argv+argc, "--quiet")) {
            logging::disable();
        }
        if (argparse::cmdOptionExists(argv, argv+argc, "--debug")) {
            logging::enableDebug();
        }
        if (argparse::cmdOptionExists(argv, argv+argc, "--verbose")) {
            logging::enableVerbose();
        }
        if (argparse::cmdOptionExists(argv, argv+argc, "-h") || argparse::cmdOptionExists(argv, argv+argc, "--help")) {
            printUsage(argv[0]);
            return 0;
        } 
        
        char* fsRootArg = argparse::getCmdOption(argv, argv+argc, "--fsroot");
        std::string fsRoot = fsRootArg ? std::string(fsRootArg) : "/";
        
        if (argc < 2 || argv[1][0] == '-') { //if no arguments, or if first argument (and therefore all args) is an option
            //printUsage(argv[0]);
            serialFileName = defaultSerialFile;
        } else {
            serialFileName = std::string(argv[1]);
            if (argc >2 && argv[2][0] != '-') { //second argument is for the output file
                outFile = std::string(argv[2]);
            }
        }
        
        //prevent page-swaps to increase performace:
        int retval = mlockall(MCL_FUTURE|MCL_CURRENT);
        if (retval) {
            LOGW("Warning: mlockall (prevent memory swaps) in main.cpp::main() returned non-zero: %i\n", retval);
        }
        
        //Open the serial device:
        LOG("Serial file: %s\n", serialFileName.c_str());
        gparse::Com com = gparse::Com(serialFileName, outFile);
        
        //instantiate main driver:
        machines::MACHINE driver;
        
        LOG("Filesystem root: %s\n", fsRoot.c_str());
        FileSystem fs(fsRoot);
        
        //if input is stdin, or a two-way pipe, then it likely means we want to keep this input channel forever,
        //  whereas if it's a gcode file, then calls to M32 (print from file) should pause the original input file
        State<machines::MACHINE> state(driver, fs, com, com.hasWriteFile() || serialFileName == "/dev/stdin");
        
        state.eventLoop();
        return 0;
    #endif
}

int main(int argc, char** argv) {
    try { //wrap in a try/catch loop so we can safely clean up (disable IOs)
        return main_(argc, argv);
    } catch (const std::exception *e) {
        LOGE("caught std::exception*: %s. ... Exiting\n", e->what());
        #ifdef CLEAN_EXIT
            return 1; //don't rethrow exceptions; return an error code instead
        #endif
        throw;
    } catch (const std::exception &e) {
        LOGE("caught std::exception&: %s. ... Exiting\n", e.what());
        #ifdef CLEAN_EXIT
            return 1; //don't rethrow exceptions; return an error code instead
        #endif
        throw;
    } catch (...) {
        LOGE("caught unknown exception. Exiting\n");
        #ifdef CLEAN_EXIT
            return 1; //don't rethrow exceptions; return an error code instead
        #endif
        throw;
    }
}



SCENARIO("State will respond correctly to gcode commands", "[state]") {
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

        std::thread eventThread([](){ 
            machines::MACHINE driver;
            FileSystem fs("/tmp/");
            gparse::Com com = gparse::Com("PRINTIPI_TEST_INPUT", "PRINTIPI_TEST_OUTPUT");
            State<machines::MACHINE> state(driver, fs, com, true);
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

        bool hasExited = false;
        auto exitOnce = [&]() {
            if (!hasExited) {
                inputFile << "M0\n";
                REQUIRE(readLine() == "ok");
                eventThread.join();
                hasExited = true;
            }
        };

        //each WHEN case corresponds to a single test;
        //the above setup code and the teardown code further below are re-run for EVERY 'when' case.
        WHEN("The command to home axis, G28, is sent") {
            inputFile << "G28\n";
            THEN("It should be acknowledged with 'ok'") {
                REQUIRE(readLine() == "ok");
            }
        }

        //Teardown code:
        exitOnce();
    }
}