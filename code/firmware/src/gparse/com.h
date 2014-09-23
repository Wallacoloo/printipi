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
#include "response.h"

#define NO_HANDLE -1 //null file descriptor


namespace gparse {

class Com {
	int _readFd;
	int _writeFd;
	std::string _pending;
	Command _parsed;
	public:
		Com();
		Com(const std::string &fileR);
		Com(const std::string &fileR, const std::string &fileW);
		//returns true if there is a command ready to be interpreted.
		bool tendCom();
		Command getCommand() const;
		//void reply(const Command &response);
		void reply(const std::string &resp);
		void reply(const Response &resp);
		inline bool hasReadFile() const {
		    return _readFd != NO_HANDLE;
		}
	    inline bool hasWriteFile() const {
	        return _writeFd != NO_HANDLE;
	    }
			
};

};
#endif
