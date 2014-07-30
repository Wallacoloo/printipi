#ifndef ARGPARSE_H
#define ARGPARSE_H

/* 
 * Printipi/argparse.h
 * based on a stackoverflow post
 *
 * argparse provides some basic functions for parsing command line options.
 */

#include <string>

namespace argparse {

char* getCmdOption(char ** begin, char ** end, const std::string &option);
bool cmdOptionExists(char ** begin, char ** end, const std::string &option);

}
#endif
