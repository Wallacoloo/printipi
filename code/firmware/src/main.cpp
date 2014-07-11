/* 
 * Printipi
 * (c) 2014 Colin Wallace
 */

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
 *  Solder thermistor
 *  Add thermister support
 *  Add endstop support
 *  Add max/min bounds for each axis
 *  Add acceleration
 *  Add IO deactivation upon exit
 *  Add ability to put steppers to rest upon idle
 *  Make scheduler more resistant to skipping (make it so it can only run at up to, say, 2x speed to catch up. This minimizes missed steps)
 *  Investigate single-threading for entire process.
 *  Look into coordinate rounding for State::queueMovement
 *  Consider names: piprint (taken), printchef (exists), Rasprintian (play on raspbian), Rasprint, Rasprinti, Printipi
*/

#include <string>
#include <fcntl.h> //needed for (file) open()
//#include <stdlib.h> //needed for exit()
#include <sys/mman.h> //for mlockall
//#include <cstdlib> //for atexit
#include "logging.h"

#include "gparse/serial.h"
#include "state.h"
#include "drivers/driver.h"
#include "drivers/kossel/kossel.h"
#include "argparse.h"

void printUsage(char* cmd) {
	//#ifndef NO_USAGE_INFO
	LOGE("usage: %s ttyFile\n", cmd);
    //std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    //#endif
    //exit(1);
}

/*void onExit() {
	LOG("Exiting\n");
}*/

int main(int argc, char** argv) {
	/*if (DO_LOG) {
		std::atexit(onExit);
	}*/
	if (argparse::cmdOptionExists(argv, argv+argc, "--quiet")) {
    	logging::disable();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "--verbose")) {
    	logging::enableVerbose();
    }
    if (argc < 2 || argparse::cmdOptionExists(argv, argv+argc, "-h") || argparse::cmdOptionExists(argv, argv+argc, "--help")) {
        printUsage(argv[0]);
        return 1;
    }
    
    //prevent page-swaps to increase performace:
    int retval = mlockall(MCL_FUTURE|MCL_CURRENT);
    if (retval) {
    	LOGW("Warning: mlockall (prevent memory swaps) in main.cpp::main() returned non-zero: %i\n", retval);
    }
    
    char* serialFileName = argv[1];
    LOG("Serial file: %s\n", serialFileName);
    int fd = open(serialFileName, O_RDWR);
    
    drv::Kossel driver;
	State<drv::Kossel> gState(driver);
    
    gparse::comLoop(fd, gState);
    return 0;
}
