#include "com.h"

#include <unistd.h> //for (file) read() and write()
#include <fcntl.h> //needed for (file) open()
//#include "common/logging.h"

namespace gparse {

Com::Com() : _readFd(NO_HANDLE), _writeFd(NO_HANDLE) {}
Com::Com(const std::string &fileR) : _readFd(open(fileR.c_str(), O_RDWR | O_NONBLOCK)), _writeFd(NO_HANDLE) {}
Com::Com(const std::string &fileR, const std::string &fileW) 
  : _readFd(open(fileR.c_str(), O_RDWR | O_NONBLOCK)), _writeFd(open(fileW.c_str(), O_RDWR | O_NONBLOCK)) {}

bool Com::tendCom() {
	if (!_parsed.empty()) { 
		return true;
	}
	char chr;
	while (read(_readFd, &chr, 1) == 1) { //read all characters available on serial line.
		if (chr == '\n') {
			_parsed = Command(_pending);
			/*if (!NO_LOG_M105 || !_parsed.isM105()) {
				LOG("command: %s\n", _pending.c_str());
			}*/
			_pending = "";
			return !_parsed.empty(); //it's possible we got a blank line, or a comment.
		} else if (chr != '\r') {
			_pending += chr;
		}
	}
	return false;
}

Command Com::getCommand() const {
	return _parsed;
}

void Com::reply(const std::string &resp) {
	if (hasWriteFile()) {
	    /*ssize_t res = */ write(_writeFd, resp.c_str(), resp.length());
	}
	_parsed = Command();
}

void Com::reply(const Response &resp) {
    reply(resp.toString());
}


}
