#ifndef GPARSE_SERIAL_H
#define GPARSE_SERIAL_H

#include <string>
#include <unistd.h> //for (file) read() and write()
#include "command.h"
#include "state.h"

namespace gparse {

std::string readLine(int fd);
template <typename T> void comLoop(int fd, State& state, T& driver) {
	std::string cmd;
	while (1) {
		cmd = readLine(fd);
		printf("command: %s\n", cmd.c_str());
		Command parsed = Command(cmd);
		printf("parsed: %s", parsed.toGCode().c_str());
		//printf("size of command: %i\n", parsed.pieces.size());
		Command response = state.execute(parsed, driver);
		std::string resp = response.toGCode();
		printf("response: %s", resp.c_str());
		write(fd, resp.c_str(), resp.length());
	}
}


}
#endif
