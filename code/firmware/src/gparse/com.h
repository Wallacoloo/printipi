#ifndef GPARSE_COM_H
#define GPARSE_COM_H

/* 
 * Printipi/gparse/com.h
 * (c) 2014 Colin Wallace
 *
 * Com manages the low-level interfacing with whatever is controlling this printer.
 * reads are non-blocking, so tendCom() must be called on a regular basis.
 * once tendCom returns true, then a command is available via getCommand(), and a reply can be sent to the host via reply(...)
 *
 * Communication is typically done over a serial interface, but Com accepts any file descriptor,
 *   so communication can be done via stdin (/dev/stdin), or perhaps commands can be directly fed from a gcode file (untested).
 */

#include <string>
#include "command.h"


namespace gparse {

class Com {
	int _fd;
	std::string _pending;
	Command _parsed;
	public:
		Com() {}
		Com(const std::string &file);
		//returns true if there is a command ready to be interpreted.
		bool tendCom();
		Command getCommand() const;
		void reply(const Command &response) const;
			
};

};
#endif
