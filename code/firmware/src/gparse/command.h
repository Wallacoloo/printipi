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
		bool hasX() const;
		bool hasY() const;
		bool hasZ() const;
		bool hasE() const;
		bool hasAnyXYZParam() const;
		bool hasAnyXYZEParam() const;
		inline bool isG1()   const { return this->opcode ==   "G1"; }
		inline bool isG20()  const { return this->opcode ==  "G20"; }
		inline bool isG21()  const { return this->opcode ==  "G21"; }
		inline bool isG28()  const { return this->opcode ==  "G28"; }
		inline bool isG90()  const { return this->opcode ==  "G90"; }
		inline bool isG91()  const { return this->opcode ==  "G91"; }
		inline bool isG92()  const { return this->opcode ==  "G92"; }
		inline bool isM21()  const { return this->opcode ==  "M21"; }
		inline bool isM82()  const { return this->opcode ==  "M82"; }
		inline bool isM83()  const { return this->opcode ==  "M83"; }
		inline bool isM104() const { return this->opcode == "M104"; }
		inline bool isM105() const { return this->opcode == "M105"; }
		inline bool isM106() const { return this->opcode == "M106"; }
		inline bool isM107() const { return this->opcode == "M107"; }
		inline bool isM109() const { return this->opcode == "M109"; }
		inline bool isM110() const { return this->opcode == "M110"; }
		inline bool isTxxx() const { return this->opcode.length() && this->opcode[0] == 'T'; }
	private:
		void addPieceOrOpcode(std::string const& piece);
};

}
#endif
