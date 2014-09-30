#ifndef COMMAND_H
#define COMMAND_H

/* 
 * Printipi/gparse/command.h
 * (c) 2014 Colin Wallace
 *
 * Command objects represent a single line of gcode.
 * They can be parsed from a string, and then can be queried by opcode and parameters.
 * They can also be constructed and used as replies to the host (although this may change in future implementations)
 */

#include <string>
#include <vector>
#include <array>
#include <cstdint> //for uint32_t
#include <cmath> //for NAN

/*#List of commands on Reprap Wiki:
cmds = ['G0', 'G1', 'G2', 'G3', 'G4', 'G10', 'G20', 'G21', 'G28', 'G29', 'G30', 'G31', 'G32', 'G90', 'G91', 'G92', 'M0', 'M1', 'M3', 'M4', 'M5', 'M7', 'M8', 'M9', 'M10', 'M11', 'M17', 'M18', 'M20', 'M21', 'M22', 'M23', 'M24', 'M25', 'M26', 'M27', 'M28', 'M29', 'M30', 'M32', 'M40', 'M41', 'M42', 'M43', 'M80', 'M81', 'M82', 'M83', 'M84', 'M92', 'M98', 'M99', 'M103', 'M104', 'M105', 'M106', 'M107', 'M108', 'M109', 'M110', 'M111', 'M112', 'M113', 'M114', 'M115', 'M116', 'M117', 'M118', 'M119', 'M120', 'M121', 'M122', 'M123', 'M124', 'M126', 'M127', 'M128', 'M129', 'M130', 'M131', 'M132', 'M133', 'M134', 'M135', 'M136', 'M140', 'M141', 'M142', 'M143', 'M144', 'M160', 'M190', 'M200', 'M201', 'M202', 'M203', 'M204', 'M205', 'M206', 'M207', 'M208', 'M209', 'M210', 'M220', 'M221', 'M226', 'M227', 'M228', 'M229', 'M230', 'M240', 'M241', 'M245', 'M246', 'M280', 'M300', 'M301', 'M302', 'M303', 'M304', 'M305', 'M400', 'M420', 'M540', 'M550', 'M551', 'M552', 'M553', 'M554', 'M555', 'M556', 'M557', 'M558', 'M559', 'M560', 'M561', 'M562', 'M563', 'M564', 'M565', 'M566', 'M567', 'M568', 'M569', 'M665', 'M906', 'M998', 'M999']
#code to generate isXXXX() functions:
literals = ["0x%su" %cmd.encode("hex") for cmd in cmds]
funcs = ["inline bool is%s() const { return isOpcode(%s); }" %(cmd, enc) for (cmd, enc) in zip(cmds, literals)]
pretty = "\n".join("        %s" %f for f in funcs)
*/

namespace gparse {

class Command {
    public:
    //std::string opcode;
    uint32_t opcodeStr; //opcode still encoded as a 4-character string. MSB=first char, LSB=last char. String is right-adjusted (ie, the MSBs are 0 in the case that opcode isn't full 4 characters).
    std::vector<std::string> pieces; //the command when split on spaces. Eg "G1 X2 Y3" -> ["G1", "X2", "Y3"]
    std::array<float, 26> arguments; //26 alphabetic possible arguments per Gcode. Case insensitive. Internally, this will default to NaN
    public:
        //initialize the command object from a line of GCode
        inline Command() : opcodeStr(0), arguments({NAN}) {}
        Command(std::string const&);
        inline bool empty() const {
            return opcodeStr == 0;
            //return opcode.empty();
        }
        std::string getOpcode() const;
        std::string toGCode() const;
        bool hasParam(char label) const;
        
        std::string getStrParam(char label, bool &hasParam) const;
        inline std::string getStrParam(char label) const {
            bool _ignore;
            return getStrParam(label, _ignore);
        }
        
        float getFloatParam(char label, float def, bool &hasParam) const;
        inline float getFloatParam(char label, float def=NAN) const {
            bool _ignore;
            return getFloatParam(label, def, _ignore);
        }
        inline float getFloatParam(char label, bool &hasParam) const {
            return getFloatParam(label, NAN, hasParam);
        }
        
        inline float getX(float def=NAN) const {
            return getFloatParam('C', def);
        }
        inline float getX(bool &hasParam) const {
            return getFloatParam('X', hasParam);
        }
        inline float getY(float def=NAN) const {
            return getFloatParam('Y', def);
        }
        inline float getY(bool &hasParam) const {
            return getFloatParam('Y', hasParam);
        }
        inline float getZ(float def=NAN) const {
            return getFloatParam('Z', def);
        }
        inline float getZ(bool &hasParam) const {
            return getFloatParam('Z', hasParam);
        }
        inline float getE(float def=NAN) const { //extrusion distance
            return getFloatParam('E', def);
        }
        inline float getE(bool &hasParam) const {
            return getFloatParam('E', hasParam);
        }
        inline float getF(float def=NAN) const { //extruder feed-rate.
            return getFloatParam('F', def);
        }
        inline float getF(bool &hasParam) const {
            return getFloatParam('F', hasParam);
        }
        inline float getS(float def=NAN) const { //PWM rate
            return getFloatParam('S', def);
        }
        inline float getS(bool &hasParam) const {
            return getFloatParam('S', hasParam);
        }
        inline bool hasX() const {
            return hasParam('X');
        }
        inline bool hasY() const {
            return hasParam('Y');
        }
        inline bool hasZ() const {
            return hasParam('Z');
        }
        inline bool hasE() const {
            return hasParam('E');
        }
        inline bool hasF() const {
            return hasParam('F');
        }
        inline bool hasS() const {
            return hasParam('S');
        }
        inline bool hasAnyXYZParam() const {
            return hasX() || hasY() || hasZ();
        }
        inline bool hasAnyXYZEParam() const {
            return hasAnyXYZParam() || hasE();
        }
        inline bool isG0() const { return isOpcode(0x4730u); }
        inline bool isG1() const { return isOpcode(0x4731u); }
        inline bool isG2() const { return isOpcode(0x4732u); }
        inline bool isG3() const { return isOpcode(0x4733u); }
        inline bool isG4() const { return isOpcode(0x4734u); }
        inline bool isG10() const { return isOpcode(0x473130u); }
        inline bool isG20() const { return isOpcode(0x473230u); }
        inline bool isG21() const { return isOpcode(0x473231u); }
        inline bool isG28() const { return isOpcode(0x473238u); }
        inline bool isG29() const { return isOpcode(0x473239u); }
        inline bool isG30() const { return isOpcode(0x473330u); }
        inline bool isG31() const { return isOpcode(0x473331u); }
        inline bool isG32() const { return isOpcode(0x473332u); }
        inline bool isG90() const { return isOpcode(0x473930u); }
        inline bool isG91() const { return isOpcode(0x473931u); }
        inline bool isG92() const { return isOpcode(0x473932u); }
        inline bool isM0() const { return isOpcode(0x4d30u); }
        inline bool isM1() const { return isOpcode(0x4d31u); }
        inline bool isM3() const { return isOpcode(0x4d33u); }
        inline bool isM4() const { return isOpcode(0x4d34u); }
        inline bool isM5() const { return isOpcode(0x4d35u); }
        inline bool isM7() const { return isOpcode(0x4d37u); }
        inline bool isM8() const { return isOpcode(0x4d38u); }
        inline bool isM9() const { return isOpcode(0x4d39u); }
        inline bool isM10() const { return isOpcode(0x4d3130u); }
        inline bool isM11() const { return isOpcode(0x4d3131u); }
        inline bool isM17() const { return isOpcode(0x4d3137u); }
        inline bool isM18() const { return isOpcode(0x4d3138u); }
        inline bool isM20() const { return isOpcode(0x4d3230u); }
        inline bool isM21() const { return isOpcode(0x4d3231u); }
        inline bool isM22() const { return isOpcode(0x4d3232u); }
        inline bool isM23() const { return isOpcode(0x4d3233u); }
        inline bool isM24() const { return isOpcode(0x4d3234u); }
        inline bool isM25() const { return isOpcode(0x4d3235u); }
        inline bool isM26() const { return isOpcode(0x4d3236u); }
        inline bool isM27() const { return isOpcode(0x4d3237u); }
        inline bool isM28() const { return isOpcode(0x4d3238u); }
        inline bool isM29() const { return isOpcode(0x4d3239u); }
        inline bool isM30() const { return isOpcode(0x4d3330u); }
        inline bool isM32() const { return isOpcode(0x4d3332u); }
        inline bool isM40() const { return isOpcode(0x4d3430u); }
        inline bool isM41() const { return isOpcode(0x4d3431u); }
        inline bool isM42() const { return isOpcode(0x4d3432u); }
        inline bool isM43() const { return isOpcode(0x4d3433u); }
        inline bool isM80() const { return isOpcode(0x4d3830u); }
        inline bool isM81() const { return isOpcode(0x4d3831u); }
        inline bool isM82() const { return isOpcode(0x4d3832u); }
        inline bool isM83() const { return isOpcode(0x4d3833u); }
        inline bool isM84() const { return isOpcode(0x4d3834u); }
        inline bool isM92() const { return isOpcode(0x4d3932u); }
        inline bool isM98() const { return isOpcode(0x4d3938u); }
        inline bool isM99() const { return isOpcode(0x4d3939u); }
        inline bool isM103() const { return isOpcode(0x4d313033u); }
        inline bool isM104() const { return isOpcode(0x4d313034u); }
        inline bool isM105() const { return isOpcode(0x4d313035u); }
        inline bool isM106() const { return isOpcode(0x4d313036u); }
        inline bool isM107() const { return isOpcode(0x4d313037u); }
        inline bool isM108() const { return isOpcode(0x4d313038u); }
        inline bool isM109() const { return isOpcode(0x4d313039u); }
        inline bool isM110() const { return isOpcode(0x4d313130u); }
        inline bool isM111() const { return isOpcode(0x4d313131u); }
        inline bool isM112() const { return isOpcode(0x4d313132u); }
        inline bool isM113() const { return isOpcode(0x4d313133u); }
        inline bool isM114() const { return isOpcode(0x4d313134u); }
        inline bool isM115() const { return isOpcode(0x4d313135u); }
        inline bool isM116() const { return isOpcode(0x4d313136u); }
        inline bool isM117() const { return isOpcode(0x4d313137u); }
        inline bool isM118() const { return isOpcode(0x4d313138u); }
        inline bool isM119() const { return isOpcode(0x4d313139u); }
        inline bool isM120() const { return isOpcode(0x4d313230u); }
        inline bool isM121() const { return isOpcode(0x4d313231u); }
        inline bool isM122() const { return isOpcode(0x4d313232u); }
        inline bool isM123() const { return isOpcode(0x4d313233u); }
        inline bool isM124() const { return isOpcode(0x4d313234u); }
        inline bool isM126() const { return isOpcode(0x4d313236u); }
        inline bool isM127() const { return isOpcode(0x4d313237u); }
        inline bool isM128() const { return isOpcode(0x4d313238u); }
        inline bool isM129() const { return isOpcode(0x4d313239u); }
        inline bool isM130() const { return isOpcode(0x4d313330u); }
        inline bool isM131() const { return isOpcode(0x4d313331u); }
        inline bool isM132() const { return isOpcode(0x4d313332u); }
        inline bool isM133() const { return isOpcode(0x4d313333u); }
        inline bool isM134() const { return isOpcode(0x4d313334u); }
        inline bool isM135() const { return isOpcode(0x4d313335u); }
        inline bool isM136() const { return isOpcode(0x4d313336u); }
        inline bool isM140() const { return isOpcode(0x4d313430u); }
        inline bool isM141() const { return isOpcode(0x4d313431u); }
        inline bool isM142() const { return isOpcode(0x4d313432u); }
        inline bool isM143() const { return isOpcode(0x4d313433u); }
        inline bool isM144() const { return isOpcode(0x4d313434u); }
        inline bool isM160() const { return isOpcode(0x4d313630u); }
        inline bool isM190() const { return isOpcode(0x4d313930u); }
        inline bool isM200() const { return isOpcode(0x4d323030u); }
        inline bool isM201() const { return isOpcode(0x4d323031u); }
        inline bool isM202() const { return isOpcode(0x4d323032u); }
        inline bool isM203() const { return isOpcode(0x4d323033u); }
        inline bool isM204() const { return isOpcode(0x4d323034u); }
        inline bool isM205() const { return isOpcode(0x4d323035u); }
        inline bool isM206() const { return isOpcode(0x4d323036u); }
        inline bool isM207() const { return isOpcode(0x4d323037u); }
        inline bool isM208() const { return isOpcode(0x4d323038u); }
        inline bool isM209() const { return isOpcode(0x4d323039u); }
        inline bool isM210() const { return isOpcode(0x4d323130u); }
        inline bool isM220() const { return isOpcode(0x4d323230u); }
        inline bool isM221() const { return isOpcode(0x4d323231u); }
        inline bool isM226() const { return isOpcode(0x4d323236u); }
        inline bool isM227() const { return isOpcode(0x4d323237u); }
        inline bool isM228() const { return isOpcode(0x4d323238u); }
        inline bool isM229() const { return isOpcode(0x4d323239u); }
        inline bool isM230() const { return isOpcode(0x4d323330u); }
        inline bool isM240() const { return isOpcode(0x4d323430u); }
        inline bool isM241() const { return isOpcode(0x4d323431u); }
        inline bool isM245() const { return isOpcode(0x4d323435u); }
        inline bool isM246() const { return isOpcode(0x4d323436u); }
        inline bool isM280() const { return isOpcode(0x4d323830u); }
        inline bool isM300() const { return isOpcode(0x4d333030u); }
        inline bool isM301() const { return isOpcode(0x4d333031u); }
        inline bool isM302() const { return isOpcode(0x4d333032u); }
        inline bool isM303() const { return isOpcode(0x4d333033u); }
        inline bool isM304() const { return isOpcode(0x4d333034u); }
        inline bool isM305() const { return isOpcode(0x4d333035u); }
        inline bool isM400() const { return isOpcode(0x4d343030u); }
        inline bool isM420() const { return isOpcode(0x4d343230u); }
        inline bool isM540() const { return isOpcode(0x4d353430u); }
        inline bool isM550() const { return isOpcode(0x4d353530u); }
        inline bool isM551() const { return isOpcode(0x4d353531u); }
        inline bool isM552() const { return isOpcode(0x4d353532u); }
        inline bool isM553() const { return isOpcode(0x4d353533u); }
        inline bool isM554() const { return isOpcode(0x4d353534u); }
        inline bool isM555() const { return isOpcode(0x4d353535u); }
        inline bool isM556() const { return isOpcode(0x4d353536u); }
        inline bool isM557() const { return isOpcode(0x4d353537u); }
        inline bool isM558() const { return isOpcode(0x4d353538u); }
        inline bool isM559() const { return isOpcode(0x4d353539u); }
        inline bool isM560() const { return isOpcode(0x4d353630u); }
        inline bool isM561() const { return isOpcode(0x4d353631u); }
        inline bool isM562() const { return isOpcode(0x4d353632u); }
        inline bool isM563() const { return isOpcode(0x4d353633u); }
        inline bool isM564() const { return isOpcode(0x4d353634u); }
        inline bool isM565() const { return isOpcode(0x4d353635u); }
        inline bool isM566() const { return isOpcode(0x4d353636u); }
        inline bool isM567() const { return isOpcode(0x4d353637u); }
        inline bool isM568() const { return isOpcode(0x4d353638u); }
        inline bool isM569() const { return isOpcode(0x4d353639u); }
        inline bool isM665() const { return isOpcode(0x4d363635u); }
        inline bool isM906() const { return isOpcode(0x4d393036u); }
        inline bool isM998() const { return isOpcode(0x4d393938u); }
        inline bool isM999() const { return isOpcode(0x4d393939u); }
        inline bool isTxxx() const { return isFirstChar('T'); }
    private:
        inline void addPiece(std::string const& piece) {
            this->pieces.push_back(piece);
        }
        inline bool isOpcode(uint32_t op) const {
            return opcodeStr == op;
        }
        bool isFirstChar(char c) const;
};

}
#endif
