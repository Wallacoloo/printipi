#ifndef GPARSE_STATE_H
#define GPARSE_STATE_H

#include <string>
#include "command.h"

namespace gparse {

enum PositionMode {
	POS_ABSOLUTE,
	POS_RELATIVE
};

class State {
	PositionMode positionMode = POS_ABSOLUTE;
	public:
		static const std::string OP_G90 ;//  = "G90";
		static const std::string OP_G91 ;//  = "G91";
		static const std::string OP_M21 ;//  = "M21";
		static const std::string OP_M105;// = "M105";
		static const std::string OP_M110;// = "M110";
		State() {}
		void setPositionMode(PositionMode mode);
		/*execute the GCode on a Driver object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		template <typename T> Command execute(Command const& cmd, T &driver) {
			std::string opcode = cmd.getOpcode();
			Command resp;
			if (opcode == OP_G90) {
				setPositionMode(POS_ABSOLUTE);
			} else if (opcode == OP_G91) {
				setPositionMode(POS_RELATIVE);
			} else if (opcode == OP_M21) { //initialize SD card.
				resp = Command::OK;
			} else if (opcode == OP_M105) { //get temperature
				int t, b;
				driver.getTemperature(t, b);
				resp = Command("ok T:" + std::to_string(t) + " B:" + std::to_string(b));
			} else if (opcode == OP_M110) { //set current line number
				resp = Command::OK;
			} else {
				throw new std::string("unrecognized gcode opcode");
			}
			return resp;
		}
};

}
#endif
