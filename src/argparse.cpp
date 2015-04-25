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

#include "argparse.h"

#include <algorithm>

namespace argparse {

char** getCmdOptionPtr(char ** begin, char ** end, const std::string &option) {
	char ** itr = std::find(begin, end, option);
    if (itr != end) {
        return itr;
    }
    return NULL;
}

char* getArgumentForCmdOption(char ** begin, char ** end, const std::string &option) {
	char **itr = getCmdOptionPtr(begin, end, option);
	if (itr && ++itr != end) {
		return *itr;
	}
	return NULL;
}

int getCmdOptionIdx(char ** begin, char ** end, const std::string &option, int dflt) {
	char ** ptr = getCmdOptionPtr(begin, end, option);
	if (!ptr) {
		return dflt;
	}
	return (ptr - begin);
}


bool cmdOptionExists(char** begin, char** end, const std::string& option) {
    return std::find(begin, end, option) != end;
}

}
