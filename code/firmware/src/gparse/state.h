#ifndef GPARSE_STATE_H
#define GPARSE_STATE_H

namespace gparse {

enum PositionMode {
	POS_ABSOLUTE,
	POS_RELATIVE
};

class State {
	PositionMode positionMode = POS_ABSOLUTE;
	public:
		State() {}
		void setPositionMode(PositionMode mode);
		/*execute the GCode on a target object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		template <typename T> Command execute(Command const& cmd, T &target) {
			std::string opcode = cmd.getOpcode();
			Command resp;
			if (opcode == "G90") {
				setPositionMode(POS_ABSOLUTE);
			} else if (opcode == "G91") {
				setPositionMode(POS_RELATIVE);
			} else if (opcode == "M21") { //initialize SD card.
				resp = Command::OK;
			} else if (opcode == "M105") { //get temperature
				int t, b;
				target.getTemperature(t, b);
				resp = Command("ok T:" + std::to_string(t) + " B:" + std::to_string(b));
			} else if (opcode == "M110") { //set current line number
				resp = Command::OK;
			} else {
				throw new std::string("unrecognized gcode opcode");
			}
			return resp;
		}
};

}
#endif
