#include "logging.h"

namespace logging {

#if DO_LOG == 1

bool _info = true;
bool _verbose = false;

bool isInfoEnabled() {
	return _info;
}
bool isVerboseEnabled() {
	return _verbose;
}

void disable() {
	_info = false;
	_verbose = false;
}

void enableVerbose() {
	_verbose = true;
	LOGV("verbose logging enabled\n");
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
