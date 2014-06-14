#include <iostream> //for std::cerr?
#include <string>
#include <fcntl.h> //needed for (file) open()
#include <stdlib.h> //needed for exit()
#include <stdio.h> //for printf?

#include "gparse/serial.h"
//#include "command.h"

void printUsageAndQuit(char* cmd) {
    std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    exit(1);
}

int main(int argc, char** argv) {
    if (argc != 2) {
        printUsageAndQuit(argv[0]);
    }
    char* serialFileName = argv[1];
    printf("Serial file: %s\n", serialFileName);
    int fd = open(serialFileName, O_RDWR);
    gparse::readLoop(fd);
    printf("Exiting\n");
    return 0;
}
