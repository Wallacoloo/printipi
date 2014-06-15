#ifndef GPARSE_STATE_H
#define GPARSE_STATE_H

#include <string>
#include "command.h"
#include "event.h"

namespace gparse {

enum PositionMode {
	POS_ABSOLUTE,
	POS_RELATIVE
};

enum LengthUnit {
	UNIT_MM,
	UNIT_IN
};

class State {
	PositionMode positionMode = POS_ABSOLUTE;
	LengthUnit unitMode = UNIT_MM;
	std::queue<Event> eventQueue;
	public:
		static const std::string OP_G1  ;//  = "G1" ;
		static const std::string OP_G20 ;//  = "G20";
		static const std::string OP_G21 ;//  = "G21";
		static const std::string OP_G90 ;//  = "G90";
		static const std::string OP_G91 ;//  = "G91";
		static const std::string OP_M21 ;//  = "M21";
		static const std::string OP_M105;// = "M105";
		static const std::string OP_M110;// = "M110";
		State() {}
		void setPositionMode(PositionMode mode);
		void setUnitMode(LengthUnit mode);
		/*execute the GCode on a Driver object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		template <typename T> Command execute(Command const& cmd, T &driver) {
			std::string opcode = cmd.getOpcode();
			Command resp;
			if (opcode == OP_G1) { //controlled (linear) movement.
				float x = cmd.getX(); //new x-coordinate.
				float y = cmd.getY(); //new y-coordinate.
				float e = cmd.getE(); //extrusion amount.
				
			} else if (opcode == OP_G20) { //g-code coordinates will now be interpreted as inches
				setUnitMode(UNIT_IN);
				resp = Command::OK;
			} else if (opcode == OP_G21) { //g-code coordinates will now be interpreted as millimeters.
				setUnitMode(UNIT_MM);
				resp = Command::OK;
			} else if (opcode == OP_G90) { //set g-code coordinates to absolute
				setPositionMode(POS_ABSOLUTE);
				resp = Command::OK;
			} else if (opcode == OP_G91) { //set g-code coordinates to relative
				setPositionMode(POS_RELATIVE);
				resp = Command::OK;
			} else if (opcode == OP_M21) { //initialize SD card (nothing to do).
				resp = Command::OK;
			} else if (opcode == OP_M105) { //get temperature, in C
				int t=-300, b=-300; //a temperature < absolute zero means no reading available.
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
