/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ARGPARSE_H
#define ARGPARSE_H

#include <string>


/* 
 * based on a stackoverflow post
 *
 * argparse provides some basic functions for parsing command line options.
 */
namespace argparse {

// return a pointer into the argv array that matches option when dereferenced, or NULL if non-existent
char** getCmdOptionPtr(char ** begin, char ** end, const std::string &option);

// return the argument following <option>, or NULL if the option or its argument doesn't exist
char* getArgumentForCmdOption(char ** begin, char ** end, const std::string &option);

// retrieve the index of the command option in the provided argv array, or dflt if non-existent
int getCmdOptionIdx(char ** begin, char ** end, const std::string &option, int dflt=-1);

// return true if option is located in the provided argv array
bool cmdOptionExists(char ** begin, char ** end, const std::string &option);

}
#endif
