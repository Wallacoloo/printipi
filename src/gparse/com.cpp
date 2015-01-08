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

namespace gparse {

bool Com::tendCom() {
    if (!_parsed.empty()) { 
        return true;
    }
    if (!hasReadFile()) {
        return false;
    }
    //clear any eof bit possibly set previously (if a stream and not a file)
    _readFd->clear();
    char chr;
    //while (read(_readFd, &chr, 1) == 1) { //read all characters available on serial line.
    while((chr = _readFd->get()) != std::char_traits<char>::eof()) {
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
    return (bool)_readFd;
}
bool Com::hasWriteFile() const {
    return (bool)_writeFd;
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
        (*_writeFd) << respStr;
        _writeFd->put('\n');
        _writeFd->flush();
    }
    //The pending command has be replied to, so reset it.
    _parsed = Command();
}

}
