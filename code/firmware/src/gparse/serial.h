#ifndef GPARSE_SERIAL_H
#define GPARSE_SERIAL_H

#include <string>

namespace gparse {

std::string readLine(int fd);
void readLoop(int fd);

}

#endif
