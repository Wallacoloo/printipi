#include "logging.h"

namespace logging {

#if DO_LOG == 1

bool _info = true;
bool _debug = false;
bool _verbose = false;

bool isInfoEnabled() {
	return _info;
}
bool isDebugEnabled() {
	return _debug;
}
bool isVerboseEnabled() {
	return _verbose;
}

void disable() {
	_info = false;
	_debug = false;
	_verbose = false;
}

void enableDebug() {
	_debug = true;
	LOG("debug logging enabled\n");
}
void enableVerbose() {
	_verbose = true;
	LOG("verbose logging enabled\n");
}

#else

//already defined in header.
/*bool isInfoEnabled() {
	return false;
}
bool isVerboseEnabled() {
	return false;
}

void disable() {}
void enableVerbose() {}*/

#endif

}
