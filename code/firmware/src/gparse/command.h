#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <vector>
#include <queue>
#include <cmath>

//LIST OF ALL COMMANDS ON REPRAP WIKI:
//['G0', 'G1', 'G2', 'G3', 'G4', 'G10', 'G20', 'G21', 'G28', 'G29', 'G30', 'G31', 'G32', 'G90', 'G91', 'G92', 'M0', 'M1', 'M3', 'M4', 'M5', 'M7', 'M8', 'M9', 'M10', 'M11', 'M17', 'M18', 'M20', 'M21', 'M22', 'M23', 'M24', 'M25', 'M26', 'M27', 'M28', 'M29', 'M30', 'M32', 'M40', 'M41', 'M42', 'M43', 'M80', 'M81', 'M82', 'M83', 'M84', 'M92', 'M98', 'M99', 'M103', 'M104', 'M105', 'M106', 'M107', 'M108', 'M109', 'M110', 'M111', 'M112', 'M113', 'M114', 'M115', 'M116', 'M117', 'M118', 'M119', 'M120', 'M121', 'M122', 'M123', 'M124', 'M126', 'M127', 'M128', 'M129', 'M130', 'M131', 'M132', 'M133', 'M134', 'M135', 'M136', 'M140', 'M141', 'M142', 'M143', 'M144', 'M160', 'M190', 'M200', 'M201', 'M202', 'M203', 'M204', 'M205', 'M206', 'M207', 'M208', 'M209', 'M210', 'M220', 'M221', 'M226', 'M227', 'M228', 'M229', 'M230', 'M240', 'M241', 'M245', 'M246', 'M280', 'M300', 'M301', 'M302', 'M303', 'M304', 'M305', 'M400', 'M420', 'M540', 'M550', 'M551', 'M552', 'M553', 'M554', 'M555', 'M556', 'M557', 'M558', 'M559', 'M560', 'M561', 'M562', 'M563', 'M564', 'M565', 'M566', 'M567', 'M568', 'M569', 'M665', 'M906', 'M998', 'M999']

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
		inline bool isG0()   const { return this->opcode ==   "G0"; }
		inline bool isG1()   const { return this->opcode ==   "G1"; }
		inline bool isG2()   const { return this->opcode ==   "G2"; }
		inline bool isG3()   const { return this->opcode ==   "G3"; }
		inline bool isG4()   const { return this->opcode ==   "G4"; }
		inline bool isG10()  const { return this->opcode ==  "G10"; }
		inline bool isG20()  const { return this->opcode ==  "G20"; }
		inline bool isG21()  const { return this->opcode ==  "G21"; }
		inline bool isG28()  const { return this->opcode ==  "G28"; }
		inline bool isG29()  const { return this->opcode ==  "G29"; }
		inline bool isG30()  const { return this->opcode ==  "G30"; }
		inline bool isG31()  const { return this->opcode ==  "G31"; }
		inline bool isG32()  const { return this->opcode ==  "G32"; }
		inline bool isG90()  const { return this->opcode ==  "G90"; }
		inline bool isG91()  const { return this->opcode ==  "G91"; }
		inline bool isG92()  const { return this->opcode ==  "G92"; }
		inline bool isM0()   const { return this->opcode ==   "M0"; }
		inline bool isM1()   const { return this->opcode ==   "M1"; }
		inline bool isM3()   const { return this->opcode ==   "M3"; }
		inline bool isM4()   const { return this->opcode ==   "M4"; }
		inline bool isM5()   const { return this->opcode ==   "M5"; }
		inline bool isM7()   const { return this->opcode ==   "M7"; }
		inline bool isM8()   const { return this->opcode ==   "M8"; }
		inline bool isM9()   const { return this->opcode ==   "M9"; }
		inline bool isM10()  const { return this->opcode ==  "M10"; }
		inline bool isM11()  const { return this->opcode ==  "M11"; }
		inline bool isM17()  const { return this->opcode ==  "M17"; }
		inline bool isM18()  const { return this->opcode ==  "M18"; }
		inline bool isM20()  const { return this->opcode ==  "M20"; }
		inline bool isM21()  const { return this->opcode ==  "M21"; }
		inline bool isM22()  const { return this->opcode ==  "M22"; }
		inline bool isM23()  const { return this->opcode ==  "M23"; }
		inline bool isM24()  const { return this->opcode ==  "M24"; }
		inline bool isM25()  const { return this->opcode ==  "M25"; }
		inline bool isM26()  const { return this->opcode ==  "M26"; }
		inline bool isM27()  const { return this->opcode ==  "M27"; }
		inline bool isM28()  const { return this->opcode ==  "M28"; }
		inline bool isM29()  const { return this->opcode ==  "M29"; }
		inline bool isM30()  const { return this->opcode ==  "M30"; }
		inline bool isM32()  const { return this->opcode ==  "M32"; }
		inline bool isM40()  const { return this->opcode ==  "M40"; }
		inline bool isM41()  const { return this->opcode ==  "M41"; }
		inline bool isM42()  const { return this->opcode ==  "M42"; }
		inline bool isM43()  const { return this->opcode ==  "M43"; }
		inline bool isM80()  const { return this->opcode ==  "M80"; }
		inline bool isM81()  const { return this->opcode ==  "M81"; }
		inline bool isM82()  const { return this->opcode ==  "M82"; }
		inline bool isM83()  const { return this->opcode ==  "M83"; }
		inline bool isM84()  const { return this->opcode ==  "M84"; }
		inline bool isM92()  const { return this->opcode ==  "M92"; }
		inline bool isM98()  const { return this->opcode ==  "M98"; }
		inline bool isM99()  const { return this->opcode ==  "M99"; }
		inline bool isM103() const { return this->opcode == "M103"; }
		inline bool isM104() const { return this->opcode == "M104"; }
		inline bool isM105() const { return this->opcode == "M105"; }
		inline bool isM106() const { return this->opcode == "M106"; }
		inline bool isM107() const { return this->opcode == "M107"; }
		inline bool isM108() const { return this->opcode == "M108"; }
		inline bool isM109() const { return this->opcode == "M109"; }
		inline bool isM110() const { return this->opcode == "M110"; }
		inline bool isM111() const { return this->opcode == "M111"; }
		inline bool isM112() const { return this->opcode == "M112"; }
		inline bool isM113() const { return this->opcode == "M113"; }
		inline bool isM114() const { return this->opcode == "M114"; }
		inline bool isM115() const { return this->opcode == "M115"; }
		inline bool isM116() const { return this->opcode == "M116"; }
		inline bool isM117() const { return this->opcode == "M117"; }
		inline bool isM118() const { return this->opcode == "M118"; }
		inline bool isM119() const { return this->opcode == "M119"; }
		inline bool isM120() const { return this->opcode == "M120"; }
		inline bool isM121() const { return this->opcode == "M121"; }
		inline bool isM122() const { return this->opcode == "M122"; }
		inline bool isM123() const { return this->opcode == "M123"; }
		inline bool isM124() const { return this->opcode == "M124"; }
		inline bool isM126() const { return this->opcode == "M126"; }
		inline bool isM127() const { return this->opcode == "M127"; }
		inline bool isM128() const { return this->opcode == "M128"; }
		inline bool isM129() const { return this->opcode == "M129"; }
		inline bool isM130() const { return this->opcode == "M130"; }
		inline bool isM131() const { return this->opcode == "M131"; }
		inline bool isM132() const { return this->opcode == "M132"; }
		inline bool isM133() const { return this->opcode == "M133"; }
		inline bool isM134() const { return this->opcode == "M134"; }
		inline bool isM135() const { return this->opcode == "M135"; }
		inline bool isM136() const { return this->opcode == "M136"; }
		inline bool isM140() const { return this->opcode == "M140"; }
		inline bool isM141() const { return this->opcode == "M141"; }
		inline bool isM142() const { return this->opcode == "M142"; }
		inline bool isM143() const { return this->opcode == "M143"; }
		inline bool isM144() const { return this->opcode == "M144"; }
		inline bool isM160() const { return this->opcode == "M160"; }
		inline bool isM190() const { return this->opcode == "M190"; }
		inline bool isM200() const { return this->opcode == "M200"; }
		inline bool isM201() const { return this->opcode == "M201"; }
		inline bool isM202() const { return this->opcode == "M202"; }
		inline bool isM203() const { return this->opcode == "M203"; }
		inline bool isM204() const { return this->opcode == "M204"; }
		inline bool isM205() const { return this->opcode == "M205"; }
		inline bool isM206() const { return this->opcode == "M206"; }
		inline bool isM207() const { return this->opcode == "M207"; }
		inline bool isM208() const { return this->opcode == "M208"; }
		inline bool isM209() const { return this->opcode == "M209"; }
		inline bool isM210() const { return this->opcode == "M210"; }
		inline bool isM220() const { return this->opcode == "M220"; }
		inline bool isM221() const { return this->opcode == "M221"; }
		inline bool isM226() const { return this->opcode == "M226"; }
		inline bool isM227() const { return this->opcode == "M227"; }
		inline bool isM228() const { return this->opcode == "M228"; }
		inline bool isM229() const { return this->opcode == "M229"; }
		inline bool isM230() const { return this->opcode == "M230"; }
		inline bool isM240() const { return this->opcode == "M240"; }
		inline bool isM241() const { return this->opcode == "M241"; }
		inline bool isM245() const { return this->opcode == "M245"; }
		inline bool isM246() const { return this->opcode == "M246"; }
		inline bool isM280() const { return this->opcode == "M280"; }
		inline bool isM300() const { return this->opcode == "M300"; }
		inline bool isM301() const { return this->opcode == "M301"; }
		inline bool isM302() const { return this->opcode == "M302"; }
		inline bool isM303() const { return this->opcode == "M303"; }
		inline bool isM304() const { return this->opcode == "M304"; }
		inline bool isM305() const { return this->opcode == "M305"; }
		inline bool isM400() const { return this->opcode == "M400"; }
		inline bool isM420() const { return this->opcode == "M420"; }
		inline bool isM540() const { return this->opcode == "M540"; }
		inline bool isM550() const { return this->opcode == "M550"; }
		inline bool isM551() const { return this->opcode == "M551"; }
		inline bool isM552() const { return this->opcode == "M552"; }
		inline bool isM553() const { return this->opcode == "M553"; }
		inline bool isM554() const { return this->opcode == "M554"; }
		inline bool isM555() const { return this->opcode == "M555"; }
		inline bool isM556() const { return this->opcode == "M556"; }
		inline bool isM557() const { return this->opcode == "M557"; }
		inline bool isM558() const { return this->opcode == "M558"; }
		inline bool isM559() const { return this->opcode == "M559"; }
		inline bool isM560() const { return this->opcode == "M560"; }
		inline bool isM561() const { return this->opcode == "M561"; }
		inline bool isM562() const { return this->opcode == "M562"; }
		inline bool isM563() const { return this->opcode == "M563"; }
		inline bool isM564() const { return this->opcode == "M564"; }
		inline bool isM565() const { return this->opcode == "M565"; }
		inline bool isM566() const { return this->opcode == "M566"; }
		inline bool isM567() const { return this->opcode == "M567"; }
		inline bool isM568() const { return this->opcode == "M568"; }
		inline bool isM569() const { return this->opcode == "M569"; }
		inline bool isM665() const { return this->opcode == "M665"; }
		inline bool isM906() const { return this->opcode == "M906"; }
		inline bool isM998() const { return this->opcode == "M998"; }
		inline bool isM999() const { return this->opcode == "M999"; }
		inline bool isTxxx() const { return this->opcode.length() && this->opcode[0] == 'T'; }
	private:
		void addPieceOrOpcode(std::string const& piece);
};

}
#endif
