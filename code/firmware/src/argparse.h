#ifndef ARGPARSE_H
#define ARGPARSE_H

#include <string>

namespace argparse {

char* getCmdOption(char ** begin, char ** end, const std::string & option);
bool cmdOptionExists(char** begin, char** end, const std::string& option);

}
#endif
