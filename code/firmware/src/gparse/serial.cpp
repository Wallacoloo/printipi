#include "serial.h"

#include <unistd.h> //for (file) read()
#include "command.h"

namespace gparse {

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

}
