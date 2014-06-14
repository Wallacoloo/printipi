#include "serial.h"

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

}
