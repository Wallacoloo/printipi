/* 
 * Printipi/main.cpp
 * (c) 2014 Colin Wallace
 *
 * main is the program's main entry point.
 * main handles command line arguments, and instantiates the serial communications, State, and machine-specific driver.
 * Currently, the driver type must set explicitly further down in the file
 */
 
//Note: this file is great for debugging with gdb: https://gist.githubusercontent.com/skyscribe/3978082/raw/e8a0c8daec409e24b29f7c14cf74140a43a9278c/.gdbinit

/*TODO:
 * *Write SN754410 drivers
 * *Test output on one single motor
 * *Figure out extrusion in State::queueMovement.
 * *in State::queueMovement, curX, curY, curZ, etc should be recalculated from the actual axis positions to prevent drift
 *    dest*Primitive must be maintained in order to handle relative movements.
 * *Add Kossel Axis drivers
 * *Run valgrind to hunt for uninitialized variables
 * *Enable new stepper driver
 * *Wire all three steppers
 * *Add exit handlers, TO SCHEDULER.
 * *Prevent calling exit() from within an atexit handler
 * *Add 100 uF capacitors to stepper drivers (crucial!)
 * *Solder thermistor
 * *Add thermister support
 * *Add heater support (And PWM)
 * *Add endstop support
 * *Rename gmath.h to mathutil.h
 *  Prevent stepping when endstop is triggered.
 *  Add max/min bounds for each axis
 * *Add acceleration
 * *Add atexit levels, so that rpi gpio is disabled AFTER the IOdrivers are disabled.
 *    Should the exit handlers prevent double-registration (ie, use std::set)?
 *      avoid std::set just for the extra memory and code usage
 * *Add IO deactivation upon exit
 * *Add enable pin
 * *Account for PWM pins when setting Scheduler queue size.
 *  Add (configurable) absolute PWM limits to hotend, etc.
 * *Reduce number of PWM channels to what is actually needed.
 * *Make driver.numAxis a property of the CoordMap
 * *Make CoordMapT, etc return tuples instead of using writebacks
 * *Make Driver::getTemperature return tuples
 * *Allow G28 to be called multiple times
 *  Reset I part of PID control when target changes
 *  Allow M105, etc to instantly return the temperature
 * *Drop temperature readings when thread has been interrupted.
 *    Best done by checking time since last call, due to future merging of sched thread with serial reading
 * *Add ability to put steppers to rest upon idle
 *  Make scheduler more resistant to skipping (make it so it can only run at up to, say, 2x speed to catch up. This minimizes missed steps)
 * *Investigate single-threading for entire process.
 * *CoordMap::getHomePosition should return the MECHANICAL home position, instead of cartesian (simpler implementation)
 *  Remove the velocity parameter from steppers.
 *  Limit movement speed based on maximum extrusion rate.
 *  Look into coordinate rounding for State::queueMovement
 *  Consider names: piprint (taken), printchef (exists), Rasprintian (play on raspbian), Rasprint, Rasprinti, Printipi, Piface (taken)
 *  Document
 *  Put copywrite at head of every file.
 * *NO: Replace certain template parameters (eg STEPS_MM_1000 with std::ratio)
 *    Not worth it; will make code more difficult to read.
 *  Look into using DMA for more precise and accurate scheduling (see info in hotend_control.txt)
 *  Optimize gcode parser.
 *    opcodes are conveniently 4 bytes (eg M123). Can fit in one int for direct comparisons, instead of string comparisons.
 *    Can also use one single string for entire command an array of char* to mark the delimiters.
 *  rename IODriver::stepForward/backword
*/

#include <string>
#include <sys/mman.h> //for mlockall
#include "logging.h"

#include "gparse/com.h"
#include "state.h"
#include "argparse.h"

#include "drivers/kossel/kossel.h"

void printUsage(char* cmd) {
	//#ifndef NO_USAGE_INFO
	LOGE("usage: %s [ttyFile] [--help] [--quiet] [--verbose]\n", cmd);
    //std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    //#endif
    //exit(1);
}

int main_(int argc, char** argv) {
	char defaultSerialFile[] = "/dev/stdin";
	char* serialFileName;
	SchedulerBase::configureExitHandlers(); //useful to do this first-thing for catching debug info.
	if (argparse::cmdOptionExists(argv, argv+argc, "--quiet")) {
    	logging::disable();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "--verbose")) {
    	logging::enableVerbose();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "-h") || argparse::cmdOptionExists(argv, argv+argc, "--help")) {
    	printUsage(argv[0]);
    	return 0;
    } 
    if (argc < 2 || argv[1][0] == '-') { //if no arguments, or if first argument (and therefore all args) is an option
        //printUsage(argv[0]);
        serialFileName = defaultSerialFile;
        //return 1;
    } else {
    	serialFileName = argv[1];
    }
    
    //prevent page-swaps to increase performace:
    int retval = mlockall(MCL_FUTURE|MCL_CURRENT);
    if (retval) {
    	LOGW("Warning: mlockall (prevent memory swaps) in main.cpp::main() returned non-zero: %i\n", retval);
    }
    
    //Open the serial device:
    LOG("Serial file: %s\n", serialFileName);
    gparse::Com com = gparse::Com(std::string(serialFileName));
    
    //instantiate main driver:
    drv::Kossel driver;
	State<drv::Kossel> state(driver, com);
	
	state.eventLoop();
    return 0;
}

int main(int argc, char** argv) {
	try { //wrap in a try/catch loop so we can safely clean up (disable IOs)
		return main_(argc, argv);
	} catch (const std::exception *e) {
		LOGE("caught std::exception*: %s. ... Exiting\n", e->what());
		return 1;
	} catch (const std::exception &e) {
		LOGE("caught std::exception&: %s. ... Exiting\n", e.what());
		return 1;
	} catch (...) {
		LOGE("caught unknown exception. Exiting\n");
		return 1;
	}
}
