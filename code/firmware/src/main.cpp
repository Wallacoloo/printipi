#include <iostream>
#include <string>
#include <fcntl.h> //needed for (file) open()
#include <stdlib.h> //needed for exit()
#include <stdio.h>
#include <unistd.h> //for (file) read()

#include "command.h"

void printUsageAndQuit(char* cmd) {
    std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    exit(1);
}

std::string readLine(int fd) {
	std::string r;
	char chr;
	while(read(fd, &chr, 1) == 1 && chr != '\n') {
		if (chr != '\r') {
			r += chr;
			//printf("c: %c\n", chr);
		}
	}
	return r;
}

void readLoop(int fd) {
	std::string cmd;
	//std::string resp = "TEST\n";
	while (1) {
		cmd = readLine(fd);
		printf("command: %s\n", cmd.c_str());
		Command parsed = Command(cmd);
		printf("size of command: %i\n", parsed.pieces.size());
		Command response = parsed.execute("1");
		std::string resp = response.toGCode();
		write(fd, resp.c_str(), resp.length());
	}
}

int main(int argc, char** argv) {
    if (argc != 2) {
        printUsageAndQuit(argv[0]);
    }
    char* serialFileName = argv[1];
    printf("Serial file: %s\n", serialFileName);
    int fd = open(serialFileName, O_RDWR);
    readLoop(fd);
    printf("Exiting\n");
    return 0;
}
