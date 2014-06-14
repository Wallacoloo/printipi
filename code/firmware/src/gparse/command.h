#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <vector>

namespace gparse {

class Command {
	public:
	std::vector<std::string> pieces; //the command when split on spaces. Eg "G1 X2 Y3" -> ["G1", "X2", "Y3"]
	public:
		static const Command OK("ok");
		//initialize the command object from a line of GCode
		Command() {}
		Command(std::string const&);
		std::string getOpcode() const;
		std::string toGCode() const;
};

}
#endif
