#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <vector>
class Command {
	public:
	std::vector<std::string> pieces; //the command when split on spaces. Eg "G1 X2 Y3" -> ["G1", "X2", "Y3"]
	public:
		//initialize the command object from a line of GCode
		Command(std::string&);
		/*execute the GCode on a target object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		template <typename T> Command execute(T& target) {
			std::string r="TEST2";
			return Command(r);
		}
		std::string toGCode();
};


#endif
