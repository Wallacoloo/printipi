#include "serial.h"

namespace gparse {

std::string readLine(int fd) {
	std::string r;
	char chr;
	while(read(fd, &chr, 1) == 1 && chr != '\n') {
		if (chr != '\r') {
			r += chr;
		}
	}
	return r;
}

bool readLinePart(int fd, std::string &out) {
	char chr;
	int retval = read(fd, &chr, 1);
	if (retval == 1) {
		if (chr == '\n') { //end of line. Don't append the newline char.
			return true; //true; line is complete.
		} else if (chr != '\r') { //drop carriage returns.
			out += chr;
		}
		return false;
	} else {
		return false;
	}
}

}
