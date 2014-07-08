#include "logging.h"

namespace logging {

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

}
