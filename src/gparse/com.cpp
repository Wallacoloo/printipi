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
    //a call to get() may hang, especially if using a stream.
    //rdbuf().in_avail() returns the expected number of characters that can be immediately read,
    //  0 indicates: "Further calls may either retrieve more characters or return traits_type::eof()"
    // -1 indicated: "Further calls will fail (either throwing or returning 'immediately'.)"
    //  source: http://www.cplusplus.com/reference/streambuf/streambuf/in_avail/ , http://www.cplusplus.com/reference/streambuf/streambuf/showmanyc/
    //  source: http://compgroups.net/comp.lang.c+/non-blocking-file-access-possible-in-c+/1017634#5544477932267335993
    //the case of 0 is acceptable, as that is either a character or EOF
    //the case of -1
    while(_readFd->rdbuf()->in_avail() != 0 && (chr = _readFd->get()) != std::char_traits<char>::eof()) {
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
    //Only send a response if we have an output stream,
    //  the response isn't NULL,
    //  and it either isn't a comment, or it is a comment and we're configured to send comments.
    if (hasWriteFile() && !resp.isNull() && (_doSendGcodeComments || !resp.isComment())) {
        std::string respStr = resp.toString();
        (*_writeFd) << respStr;
        _writeFd->put('\n');
        _writeFd->flush();
    }
    if (!resp.isComment()) {
        //The pending command has been replied to, so reset it.
        //We must check that the response wasn't a comment, as a comment doesn't count as acknowledgement of a command.
        _parsed = Command();
    }
}

}
