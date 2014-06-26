#ifndef GPARSE_STATE_H
#define GPARSE_STATE_H

//Gcode documentation can be found:
//  http://reprap.org/wiki/G-code
//  or (with implementation): https://github.com/Traumflug/Teacup_Firmware/blob/master/gcode_process.c

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
	//PositionMode extruderPosMode = POS_RELATIVE; //set via M82 and M83
	LengthUnit unitMode = UNIT_MM;
	std::queue<Event> eventQueue;
	float _destXPrimitive, _destYPrimitive, _destZPrimitive;
	float _destEPrimitive;
	float _destFeedRatePrimitive;
	public:
	    //so-called "Primitive" units represent a cartesian coordinate from the origin, using some primitive unit (mm)
		static const int DEFAULT_HOTEND_TEMP = -300;
		static const int DEFAULT_BED_TEMP = -300;
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
		float xUnitToAbsolute(float posUnit) const;
		float yUnitToAbsolute(float posUnit) const;
		float zUnitToAbsolute(float posUnit) const;
		float eUnitToAbsolute(float posUnit) const;
		float posUnitToMM(float posUnit) const;
		float xUnitToPrimitive(float posUnit) const;
		float yUnitToPrimitive(float posUnit) const;
		float zUnitToPrimitive(float posUnit) const;
		float eUnitToPrimitive(float posUnit) const;
		float fUnitToPrimitive(float posUnit) const;
		float destXPrimitive() const; //the last queued position (X, Y, Z, E) and feed rate. Future queued commands may depend on this.
		float destYPrimitive() const;
		float destZPrimitive() const;
		float destEPrimitive() const;
		float destFeedRatePrimitive() const;
		void setDestFeedRatePrimitive(float f);
		
		void queueMovement(float curX, float curY, float curZ, float curE, float x, float y, float z, float e);
		/*execute the GCode on a Driver object that supports a well-defined interface.
		 *returns a Command to send back to the host.
		 */
		template <typename T> Command execute(Command const& cmd, T &driver) {
			std::string opcode = cmd.getOpcode();
			Command resp;
			if (opcode == OP_G1) { //controlled (linear) movement.
			    bool hasX, hasY, hasZ, hasE, hasF;
			    float curX = destXPrimitive();
			    float curY = destYPrimitive();
			    float curZ = destZPrimitive();
			    float curE = destEPrimitive();
				float x = cmd.getX(hasX); //new x-coordinate.
				float y = cmd.getY(hasY); //new y-coordinate.
				float z = cmd.getZ(hasZ); //new z-coordinate.
				float e = cmd.getE(hasE); //extrusion amount.
				float f = cmd.getF(hasF); //feed-rate.
				x = hasX ? xUnitToPrimitive(x) : curX;
				y = hasY ? yUnitToPrimitive(y) : curY;
				z = hasZ ? zUnitToPrimitive(z) : curZ;
				if (hasF) {
					this->setDestFeedRatePrimitive(fUnitToPrimitive(f));
				}
				this->queueMovement(curX, curY, curZ, curE, x, y, z, e);
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
				int t=DEFAULT_HOTEND_TEMP, b=DEFAULT_BED_TEMP; //a temperature < absolute zero means no reading available.
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
