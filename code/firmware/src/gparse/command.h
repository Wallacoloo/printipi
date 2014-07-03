#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <vector>
#include <queue>
#include <cmath>

namespace gparse {

class Command {
	public:
	std::string opcode;
	std::vector<std::string> pieces; //the command when split on spaces. Eg "G1 X2 Y3" -> ["G1", "X2", "Y3"]
	public:
		static const Command OK;
		//initialize the command object from a line of GCode
		Command() {}
		Command(std::string const&);
		std::string getOpcode() const;
		std::string toGCode() const;
		bool hasParam(char label) const;
		std::string getStrParam(char label) const;
		std::string getStrParam(char label, bool &hasParam) const;
		float getFloatParam(char label, float def, bool &hasParam) const;
		float getFloatParam(char label, float def=NAN) const;
		float getFloatParam(char label, bool &hasParam) const;
		float getX(float def=NAN) const;
		float getX(bool &hasX) const;
		float getY(float def=NAN) const;
		float getY(bool &hasY) const;
		float getZ(float def=NAN) const;
		float getZ(bool &hasZ) const;
		float getE(float def=NAN) const; //extrusion distance
		float getE(bool &hasE) const;
		float getF(float def=NAN) const; //extruder feed-rate.
		float getF(bool &hasF) const;
	private:
		void addPieceOrOpcode(std::string const& piece);
};

}
#endif
