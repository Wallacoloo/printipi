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

void Com::reply(const std::string &resp) {
    if (hasWriteFile()) {
        write(_writeFd, resp.c_str(), resp.length());
    }
    //The pending command has be replied to, so reset it.
    _parsed = Command();
}

void Com::reply(const Response &resp) {
    reply(resp.toString());
}

}
