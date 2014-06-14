#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <vector>

namespace gparse {

class Command {
	public:
	std::vector<std::string> pieces; //the command when split on spaces. Eg "G1 X2 Y3" -> ["G1", "X2", "Y3"]
	public:
		//initialize the command object from a line of GCode
		Command() {}
		Command(std::string const&);
		/*execute the GCode on a target object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		template <typename T> Command execute(T &target) {
			std::string opcode = getOpcode();
			Command resp;
			if (opcode == "M105") { //get temperature
				int t, b;
				target.getTemperature(t, b);
				resp = Command("ok T:" + std::to_string(t) + " B:" + std::to_string(b));
			} else if (opcode == "M110") { //set current line number
				resp = Command("ok");
			} else {
				throw new std::string("unrecognized gcode opcode");
			}
			return resp;
		}
		std::string getOpcode();
		std::string toGCode();
};

}
#endif
