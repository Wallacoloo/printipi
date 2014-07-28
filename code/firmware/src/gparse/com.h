#ifndef GPARSE_COM_H
#define GPARSE_COM_H

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
