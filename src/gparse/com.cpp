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

#include "com.h"

#include <unistd.h> //for (file) read() and write()
#include <fcntl.h> //needed for (file) open()

namespace gparse {

//initialize static consts:
const std::string Com::NULL_FILE_STR("/dev/null"); 

Com::Com() : _readFd(NO_HANDLE), _writeFd(NO_HANDLE), _dieOnEof(false), _isAtEof(false) {}

Com::Com(const std::string &fileR, const std::string &fileW, bool dieOnEof) 
  : _readFd(open(fileR.c_str(), O_RDWR | O_NONBLOCK))
  , _writeFd(fileW == NULL_FILE_STR ? NO_HANDLE : open(fileW.c_str(), O_RDWR | O_NONBLOCK))
  , _dieOnEof(dieOnEof)
  , _isAtEof(false) {
  }

bool Com::tendCom() {
    if (!_parsed.empty()) { 
        return true;
    }
    char chr;
    while (read(_readFd, &chr, 1) == 1) { //read all characters available on serial line.
        if (chr == '\n') {
            _parsed = Command(_pending);
            _pending = "";
            return !_parsed.empty(); //it's possible we got a blank line, or a comment.
        } else if (chr != '\r') {
            _pending += chr;
        }
    }
    //at this point, we have reached an EOF
    // we are either reading from a stream, in which case there may be more to come,
    // or we are reading from a file, in which case we should parse any pending command:
    if (_dieOnEof) {
        _isAtEof = true;
        _parsed = Command(_pending);
        _pending = "";
        return !_parsed.empty();
    }
    return false;
}

bool Com::hasReadFile() const {
    return _readFd != NO_HANDLE;
}
bool Com::hasWriteFile() const {
    return _writeFd != NO_HANDLE;
}
bool Com::isAtEof() const {
    return _isAtEof && _parsed.empty();
}
const Command& Com::getCommand() const {
    return _parsed;
}

void Com::reply(const Response &resp) {
    if (hasWriteFile() && !resp.isNull()) {
        std::string respStr = resp.toString();
        write(_writeFd, respStr.c_str(), respStr.length());
        write(_writeFd, "\n", 1);
    }
    //The pending command has be replied to, so reset it.
    _parsed = Command();
}

}
