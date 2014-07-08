//#include <iostream> //for std::cerr?
#include <string>
#include <fcntl.h> //needed for (file) open()
//#include <stdlib.h> //needed for exit()
//#include <stdio.h> //for printf?
#include <sys/mman.h> //for mlockall
#include "logging.h"

#include "gparse/serial.h"
#include "state.h"
#include "drivers/driver.h"
#include "drivers/kossel/kossel.h"
#include "argparse.h"

void printUsage(char* cmd) {
	#ifndef NO_USAGE_INFO
	LOGE("usage: %s ttyFile\n", cmd);
    //std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    #endif
    //exit(1);
}

int main(int argc, char** argv) {
    if (argc < 2 || argparse::cmdOptionExists(argv, argv+argc, "-h") || argparse::cmdOptionExists(argv, argv+argc, "--help")) {
        printUsage(argv[0]);
        return 1;
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "--quiet")) {
    	logging::disable();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "--verbose")) {
    	logging::enableVerbose();
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
    LOG("Exiting\n");
    //exit(0);
    return 0;
}
