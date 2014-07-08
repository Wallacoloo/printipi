#ifndef GPARSE_SERIAL_H
#define GPARSE_SERIAL_H

#include <string>
#include <unistd.h> //for (file) read() and write()
#include "logging.h"
#include "command.h"
//#include "state.h"

namespace gparse {

std::string readLine(int fd);
//template <typename T> void comLoop(int fd, State<T>& state) {
template <typename T> void comLoop(int fd, T& state) {
	std::string cmd;
	while (1) {
		cmd = readLine(fd);
		LOG("command: %s\n", cmd.c_str());
		Command parsed = Command(cmd);
		LOGD("parsed: %s", parsed.toGCode().c_str());
		//printf("size of command: %i\n", parsed.pieces.size());
		Command response = state.execute(parsed);
		std::string resp = response.toGCode();
		LOG("response: %s", resp.c_str());
		/*ssize_t res = */ write(fd, resp.c_str(), resp.length());
	}
}


}
#endif
