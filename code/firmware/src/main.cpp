#include <iostream> //for std::cerr?
#include <string>
#include <fcntl.h> //needed for (file) open()
//#include <stdlib.h> //needed for exit()
#include <stdio.h> //for printf?
#include "logging.h"

#include "gparse/serial.h"
#include "state.h"
#include "drivers/driver.h"
#include "drivers/kossel.h"
//#include "command.h"

drv::Kossel driver;
State<drv::Kossel> gState(driver);

void printUsage(char* cmd) {
    std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    //exit(1);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        printUsage(argv[0]);
        return 1;
    }
    char* serialFileName = argv[1];
    LOG("Serial file: %s\n", serialFileName);
    int fd = open(serialFileName, O_RDWR);
    gparse::comLoop(fd, gState);
    LOG("Exiting\n");
    //exit(0);
    return 0;
}
