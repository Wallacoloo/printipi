/* The MIT License (MIT)
 *
 * Copyright (c) 2014 Colin Wallace
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 

/*#List of commands on Reprap Wiki:
cmds = ['G0', 'G1', 'G2', 'G3', 'G4', 'G10', 'G20', 'G21', 'G28', 'G29', 'G30', 'G31', 'G32', 'G90', 'G91', 'G92', 'M0', 'M1', 'M3', 'M4', 'M5', 'M7', 'M8', 'M9', 'M10', 'M11', 'M17', 'M18', 'M20', 'M21', 'M22', 'M23', 'M24', 'M25', 'M26', 'M27', 'M28', 'M29', 'M30', 'M32', 'M40', 'M41', 'M42', 'M43', 'M80', 'M81', 'M82', 'M83', 'M84', 'M92', 'M98', 'M99', 'M103', 'M104', 'M105', 'M106', 'M107', 'M108', 'M109', 'M110', 'M111', 'M112', 'M113', 'M114', 'M115', 'M116', 'M117', 'M118', 'M119', 'M120', 'M121', 'M122', 'M123', 'M124', 'M126', 'M127', 'M128', 'M129', 'M130', 'M131', 'M132', 'M133', 'M134', 'M135', 'M136', 'M140', 'M141', 'M142', 'M143', 'M144', 'M160', 'M190', 'M200', 'M201', 'M202', 'M203', 'M204', 'M205', 'M206', 'M207', 'M208', 'M209', 'M210', 'M220', 'M221', 'M226', 'M227', 'M228', 'M229', 'M230', 'M240', 'M241', 'M245', 'M246', 'M280', 'M300', 'M301', 'M302', 'M303', 'M304', 'M305', 'M400', 'M420', 'M540', 'M550', 'M551', 'M552', 'M553', 'M554', 'M555', 'M556', 'M557', 'M558', 'M559', 'M560', 'M561', 'M562', 'M563', 'M564', 'M565', 'M566', 'M567', 'M568', 'M569', 'M665', 'M906', 'M998', 'M999']
#code to generate isXXXX() functions:
arguments = [", ".join("'%s'" %c for c in cmd) for cmd in cmds]
funcs = ["inline bool is%s() const { return isOpcode(bigEndianStr(%s)); }" %(cmd, args) for (cmd, args) in zip(cmds, arguments)]
pretty = "\n".join("        %s" %f for f in funcs)
print pretty
*/

#ifndef GPARSE_COMMAND_H
#define GPARSE_COMMAND_H

#include <string>
#include <array>
#include <cstdint> //for uint32_t
#include <cmath> //for NAN
#define GPARSE_ARG_NOT_PRESENT NAN


namespace gparse {

//internal functions - treat as private
namespace {
    //bigEndianStr turns a series of characters into a uint32_t for fast string-comparisons.
    //Eg bigEndianStr('G', '1', '0') is similar to an array, x, where x[0] == 'G', x[1] == '1' and x[2] == '0', but held in fixed-width.
    inline uint32_t bigEndianStr(char a) {
        return a;
    }
    inline uint32_t bigEndianStr(char a, char b) {
        return (a<<8) + b;
    }
    inline uint32_t bigEndianStr(char a, char b, char c) {
        return (a<<16) + (b<<8) + c;
    }
    inline uint32_t bigEndianStr(char a, char b, char c, char d) {
        return (a<<24) + (b<<16) + (c<<8) + d;
    }
}

/* 
 * Command objects represent a single line of gcode.
 * They can be parsed from a string, and then can be queried by opcode and parameters.
 */
class Command {
    public:
    //std::string opcode;
    uint32_t opcodeStr; //opcode still encoded as a 4-character string. MSB=first char, LSB=last char. String is right-adjusted (ie, the MSBs are 0 in the case that opcode isn't full 4 characters).
    //std::vector<std::string> pieces; //the command when split on spaces. Eg "G1 X2 Y3" -> ["G1", "X2", "Y3"]
    std::array<float, 26> arguments; //26 alphabetic possible arguments per Gcode. Case insensitive. Internally, this will default to NaN
    //sadly, M32, M117 and the like use an unnamed string parameter for the filename
    //I think it's relatively safe to say that there can only be one unnamed str param per gcode, as parameter order is irrelevant for all other commands, so unnamed parameters would have undefined orders.
    //  That assumption allows for significant performance benefits (ie, only one string, rather than a vector of strings)
    //  and if it turns out to be false, one can just join all the parameters into a single string with a defined delimiter (ie, a space)
    //format: M32 filename.gco
    //format: M117 Message To Display
    //both of these are valid commands, and the ONLY way to reliably parse M117 is to detect the opcode, and then store everything that follows (up until a comment) into one string.
    std::string specialStringParam;
    public:
        //default initialization. All parameters will be initialized to GPARSE_ARG_NOT_PRESENT (typically NaN)
        inline Command() : opcodeStr(0) {
            arguments.fill(GPARSE_ARG_NOT_PRESENT); //initialize all arguments to default value
        }
        //initialize the command object from a line of GCode
        Command(std::string const&);
        inline bool empty() const {
            return opcodeStr == 0;
        }
        std::string getOpcode() const;
        std::string toGCode() const;
        bool hasParam(char label) const;

        // get a param, or @def if it wasn't set in this gcode command
        float getFloatParam(char label, float def) const;
        // get a param, returning @GPARSE_ARG_NOT_PRESENT (typically null) if not explicitly set
        float getFloatParam(char label) const;
        //The specialStringParam is a filename, for M32, or a message to display, for M117.
        inline const std::string& getSpecialStringParam() const {
            return specialStringParam;
        }
        //extrusion distance
        inline float getE() const {
            return getFloatParam('E');
        }
        inline float getE(float def) const {
            return getFloatParam('E', def);
        }
        //extruder feed-rate
        inline float getF() const {
            return getFloatParam('F');
        }
        inline float getF(float def) const {
            return getFloatParam('F', def);
        }
        //arc center X coordinate
        inline float getI() const {
            return getFloatParam('I');
        }
        inline float getI(float def) const {
            return getFloatParam('I', def);
        }
        //arc center Y coordinate
        inline float getJ() const {
            return getFloatParam('J');
        }
        inline float getJ(float def) const {
            return getFloatParam('J', def);
        }
        //arc center Z coordinate
        inline float getK() const {
            return getFloatParam('K');
        }
        inline float getK(float def) const {
            return getFloatParam('K', def);
        }
        //Servo Index
        inline float getP() const {
            return getFloatParam('P');
        }
        inline float getP(float def) const {
            return getFloatParam('P', def);
        }
        //PWM rate or servo angle
        inline float getS() const {
            return getFloatParam('S');
        }
        inline float getS(float def) const {
            return getFloatParam('S', def);
        }
        // Some hosts think S should be between 0.0-1.0, others think it should be between 0-255.
        // Make a guess of the intended range based on the value & return something in the range of 0.0-1.0
        inline float getNormalizedS() const {
            return normalizeDutyCycle(getS());
        }
        inline float getNormalizedS(float def) const {
            return normalizeDutyCycle(getS(def));
        }
        inline float getX() const {
            return getFloatParam('X');
        }
        inline float getX(float def) const {
            return getFloatParam('X', def);
        }
        inline float getY() const {
            return getFloatParam('Y');
        }
        inline float getY(float def) const {
            return getFloatParam('Y', def);
        }
        inline float getZ() const {
            return getFloatParam('Z');
        }
        inline float getZ(float def) const {
            return getFloatParam('Z', def);
        }
        inline bool hasE() const {
            return hasParam('E');
        }
        inline bool hasF() const {
            return hasParam('F');
        }
        inline bool hasI() const {
            return hasParam('I');
        }
        inline bool hasJ() const {
            return hasParam('J');
        }
        inline bool hasK() const {
            return hasParam('K');
        }
        inline bool hasP() const {
            return hasParam('P');
        }
        inline bool hasS() const {
            return hasParam('S');
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
        inline bool hasAnyXYZParam() const {
            return hasX() || hasY() || hasZ();
        }
        inline bool hasAnyXYZEParam() const {
            return hasAnyXYZParam() || hasE();
        }
        //Use these routines to check gcodes, as they potentially allow more optimization than casting the opcode to a string or int.
        inline bool isG0() const { return isOpcode(bigEndianStr('G', '0')); }
        inline bool isG1() const { return isOpcode(bigEndianStr('G', '1')); }
        inline bool isG2() const { return isOpcode(bigEndianStr('G', '2')); }
        inline bool isG3() const { return isOpcode(bigEndianStr('G', '3')); }
        inline bool isG4() const { return isOpcode(bigEndianStr('G', '4')); }
        inline bool isG10() const { return isOpcode(bigEndianStr('G', '1', '0')); }
        inline bool isG20() const { return isOpcode(bigEndianStr('G', '2', '0')); }
        inline bool isG21() const { return isOpcode(bigEndianStr('G', '2', '1')); }
        inline bool isG28() const { return isOpcode(bigEndianStr('G', '2', '8')); }
        inline bool isG29() const { return isOpcode(bigEndianStr('G', '2', '9')); }
        inline bool isG30() const { return isOpcode(bigEndianStr('G', '3', '0')); }
        inline bool isG31() const { return isOpcode(bigEndianStr('G', '3', '1')); }
        inline bool isG32() const { return isOpcode(bigEndianStr('G', '3', '2')); }
        inline bool isG90() const { return isOpcode(bigEndianStr('G', '9', '0')); }
        inline bool isG91() const { return isOpcode(bigEndianStr('G', '9', '1')); }
        inline bool isG92() const { return isOpcode(bigEndianStr('G', '9', '2')); }
        inline bool isM0() const { return isOpcode(bigEndianStr('M', '0')); }
        inline bool isM1() const { return isOpcode(bigEndianStr('M', '1')); }
        inline bool isM3() const { return isOpcode(bigEndianStr('M', '3')); }
        inline bool isM4() const { return isOpcode(bigEndianStr('M', '4')); }
        inline bool isM5() const { return isOpcode(bigEndianStr('M', '5')); }
        inline bool isM7() const { return isOpcode(bigEndianStr('M', '7')); }
        inline bool isM8() const { return isOpcode(bigEndianStr('M', '8')); }
        inline bool isM9() const { return isOpcode(bigEndianStr('M', '9')); }
        inline bool isM10() const { return isOpcode(bigEndianStr('M', '1', '0')); }
        inline bool isM11() const { return isOpcode(bigEndianStr('M', '1', '1')); }
        inline bool isM17() const { return isOpcode(bigEndianStr('M', '1', '7')); }
        inline bool isM18() const { return isOpcode(bigEndianStr('M', '1', '8')); }
        inline bool isM20() const { return isOpcode(bigEndianStr('M', '2', '0')); }
        inline bool isM21() const { return isOpcode(bigEndianStr('M', '2', '1')); }
        inline bool isM22() const { return isOpcode(bigEndianStr('M', '2', '2')); }
        inline bool isM23() const { return isOpcode(bigEndianStr('M', '2', '3')); }
        inline bool isM24() const { return isOpcode(bigEndianStr('M', '2', '4')); }
        inline bool isM25() const { return isOpcode(bigEndianStr('M', '2', '5')); }
        inline bool isM26() const { return isOpcode(bigEndianStr('M', '2', '6')); }
        inline bool isM27() const { return isOpcode(bigEndianStr('M', '2', '7')); }
        inline bool isM28() const { return isOpcode(bigEndianStr('M', '2', '8')); }
        inline bool isM29() const { return isOpcode(bigEndianStr('M', '2', '9')); }
        inline bool isM30() const { return isOpcode(bigEndianStr('M', '3', '0')); }
        inline bool isM32() const { return isOpcode(bigEndianStr('M', '3', '2')); }
        inline bool isM40() const { return isOpcode(bigEndianStr('M', '4', '0')); }
        inline bool isM41() const { return isOpcode(bigEndianStr('M', '4', '1')); }
        inline bool isM42() const { return isOpcode(bigEndianStr('M', '4', '2')); }
        inline bool isM43() const { return isOpcode(bigEndianStr('M', '4', '3')); }
        inline bool isM80() const { return isOpcode(bigEndianStr('M', '8', '0')); }
        inline bool isM81() const { return isOpcode(bigEndianStr('M', '8', '1')); }
        inline bool isM82() const { return isOpcode(bigEndianStr('M', '8', '2')); }
        inline bool isM83() const { return isOpcode(bigEndianStr('M', '8', '3')); }
        inline bool isM84() const { return isOpcode(bigEndianStr('M', '8', '4')); }
        inline bool isM92() const { return isOpcode(bigEndianStr('M', '9', '2')); }
        inline bool isM98() const { return isOpcode(bigEndianStr('M', '9', '8')); }
        inline bool isM99() const { return isOpcode(bigEndianStr('M', '9', '9')); }
        inline bool isM103() const { return isOpcode(bigEndianStr('M', '1', '0', '3')); }
        inline bool isM104() const { return isOpcode(bigEndianStr('M', '1', '0', '4')); }
        inline bool isM105() const { return isOpcode(bigEndianStr('M', '1', '0', '5')); }
        inline bool isM106() const { return isOpcode(bigEndianStr('M', '1', '0', '6')); }
        inline bool isM107() const { return isOpcode(bigEndianStr('M', '1', '0', '7')); }
        inline bool isM108() const { return isOpcode(bigEndianStr('M', '1', '0', '8')); }
        inline bool isM109() const { return isOpcode(bigEndianStr('M', '1', '0', '9')); }
        inline bool isM110() const { return isOpcode(bigEndianStr('M', '1', '1', '0')); }
        inline bool isM111() const { return isOpcode(bigEndianStr('M', '1', '1', '1')); }
        inline bool isM112() const { return isOpcode(bigEndianStr('M', '1', '1', '2')); }
        inline bool isM113() const { return isOpcode(bigEndianStr('M', '1', '1', '3')); }
        inline bool isM114() const { return isOpcode(bigEndianStr('M', '1', '1', '4')); }
        inline bool isM115() const { return isOpcode(bigEndianStr('M', '1', '1', '5')); }
        inline bool isM116() const { return isOpcode(bigEndianStr('M', '1', '1', '6')); }
        inline bool isM117() const { return isOpcode(bigEndianStr('M', '1', '1', '7')); }
        inline bool isM118() const { return isOpcode(bigEndianStr('M', '1', '1', '8')); }
        inline bool isM119() const { return isOpcode(bigEndianStr('M', '1', '1', '9')); }
        inline bool isM120() const { return isOpcode(bigEndianStr('M', '1', '2', '0')); }
        inline bool isM121() const { return isOpcode(bigEndianStr('M', '1', '2', '1')); }
        inline bool isM122() const { return isOpcode(bigEndianStr('M', '1', '2', '2')); }
        inline bool isM123() const { return isOpcode(bigEndianStr('M', '1', '2', '3')); }
        inline bool isM124() const { return isOpcode(bigEndianStr('M', '1', '2', '4')); }
        inline bool isM126() const { return isOpcode(bigEndianStr('M', '1', '2', '6')); }
        inline bool isM127() const { return isOpcode(bigEndianStr('M', '1', '2', '7')); }
        inline bool isM128() const { return isOpcode(bigEndianStr('M', '1', '2', '8')); }
        inline bool isM129() const { return isOpcode(bigEndianStr('M', '1', '2', '9')); }
        inline bool isM130() const { return isOpcode(bigEndianStr('M', '1', '3', '0')); }
        inline bool isM131() const { return isOpcode(bigEndianStr('M', '1', '3', '1')); }
        inline bool isM132() const { return isOpcode(bigEndianStr('M', '1', '3', '2')); }
        inline bool isM133() const { return isOpcode(bigEndianStr('M', '1', '3', '3')); }
        inline bool isM134() const { return isOpcode(bigEndianStr('M', '1', '3', '4')); }
        inline bool isM135() const { return isOpcode(bigEndianStr('M', '1', '3', '5')); }
        inline bool isM136() const { return isOpcode(bigEndianStr('M', '1', '3', '6')); }
        inline bool isM140() const { return isOpcode(bigEndianStr('M', '1', '4', '0')); }
        inline bool isM141() const { return isOpcode(bigEndianStr('M', '1', '4', '1')); }
        inline bool isM142() const { return isOpcode(bigEndianStr('M', '1', '4', '2')); }
        inline bool isM143() const { return isOpcode(bigEndianStr('M', '1', '4', '3')); }
        inline bool isM144() const { return isOpcode(bigEndianStr('M', '1', '4', '4')); }
        inline bool isM160() const { return isOpcode(bigEndianStr('M', '1', '6', '0')); }
        inline bool isM190() const { return isOpcode(bigEndianStr('M', '1', '9', '0')); }
        inline bool isM200() const { return isOpcode(bigEndianStr('M', '2', '0', '0')); }
        inline bool isM201() const { return isOpcode(bigEndianStr('M', '2', '0', '1')); }
        inline bool isM202() const { return isOpcode(bigEndianStr('M', '2', '0', '2')); }
        inline bool isM203() const { return isOpcode(bigEndianStr('M', '2', '0', '3')); }
        inline bool isM204() const { return isOpcode(bigEndianStr('M', '2', '0', '4')); }
        inline bool isM205() const { return isOpcode(bigEndianStr('M', '2', '0', '5')); }
        inline bool isM206() const { return isOpcode(bigEndianStr('M', '2', '0', '6')); }
        inline bool isM207() const { return isOpcode(bigEndianStr('M', '2', '0', '7')); }
        inline bool isM208() const { return isOpcode(bigEndianStr('M', '2', '0', '8')); }
        inline bool isM209() const { return isOpcode(bigEndianStr('M', '2', '0', '9')); }
        inline bool isM210() const { return isOpcode(bigEndianStr('M', '2', '1', '0')); }
        inline bool isM220() const { return isOpcode(bigEndianStr('M', '2', '2', '0')); }
        inline bool isM221() const { return isOpcode(bigEndianStr('M', '2', '2', '1')); }
        inline bool isM226() const { return isOpcode(bigEndianStr('M', '2', '2', '6')); }
        inline bool isM227() const { return isOpcode(bigEndianStr('M', '2', '2', '7')); }
        inline bool isM228() const { return isOpcode(bigEndianStr('M', '2', '2', '8')); }
        inline bool isM229() const { return isOpcode(bigEndianStr('M', '2', '2', '9')); }
        inline bool isM230() const { return isOpcode(bigEndianStr('M', '2', '3', '0')); }
        inline bool isM240() const { return isOpcode(bigEndianStr('M', '2', '4', '0')); }
        inline bool isM241() const { return isOpcode(bigEndianStr('M', '2', '4', '1')); }
        inline bool isM245() const { return isOpcode(bigEndianStr('M', '2', '4', '5')); }
        inline bool isM246() const { return isOpcode(bigEndianStr('M', '2', '4', '6')); }
        inline bool isM280() const { return isOpcode(bigEndianStr('M', '2', '8', '0')); }
        inline bool isM300() const { return isOpcode(bigEndianStr('M', '3', '0', '0')); }
        inline bool isM301() const { return isOpcode(bigEndianStr('M', '3', '0', '1')); }
        inline bool isM302() const { return isOpcode(bigEndianStr('M', '3', '0', '2')); }
        inline bool isM303() const { return isOpcode(bigEndianStr('M', '3', '0', '3')); }
        inline bool isM304() const { return isOpcode(bigEndianStr('M', '3', '0', '4')); }
        inline bool isM305() const { return isOpcode(bigEndianStr('M', '3', '0', '5')); }
        inline bool isM400() const { return isOpcode(bigEndianStr('M', '4', '0', '0')); }
        inline bool isM420() const { return isOpcode(bigEndianStr('M', '4', '2', '0')); }
        inline bool isM540() const { return isOpcode(bigEndianStr('M', '5', '4', '0')); }
        inline bool isM550() const { return isOpcode(bigEndianStr('M', '5', '5', '0')); }
        inline bool isM551() const { return isOpcode(bigEndianStr('M', '5', '5', '1')); }
        inline bool isM552() const { return isOpcode(bigEndianStr('M', '5', '5', '2')); }
        inline bool isM553() const { return isOpcode(bigEndianStr('M', '5', '5', '3')); }
        inline bool isM554() const { return isOpcode(bigEndianStr('M', '5', '5', '4')); }
        inline bool isM555() const { return isOpcode(bigEndianStr('M', '5', '5', '5')); }
        inline bool isM556() const { return isOpcode(bigEndianStr('M', '5', '5', '6')); }
        inline bool isM557() const { return isOpcode(bigEndianStr('M', '5', '5', '7')); }
        inline bool isM558() const { return isOpcode(bigEndianStr('M', '5', '5', '8')); }
        inline bool isM559() const { return isOpcode(bigEndianStr('M', '5', '5', '9')); }
        inline bool isM560() const { return isOpcode(bigEndianStr('M', '5', '6', '0')); }
        inline bool isM561() const { return isOpcode(bigEndianStr('M', '5', '6', '1')); }
        inline bool isM562() const { return isOpcode(bigEndianStr('M', '5', '6', '2')); }
        inline bool isM563() const { return isOpcode(bigEndianStr('M', '5', '6', '3')); }
        inline bool isM564() const { return isOpcode(bigEndianStr('M', '5', '6', '4')); }
        inline bool isM565() const { return isOpcode(bigEndianStr('M', '5', '6', '5')); }
        inline bool isM566() const { return isOpcode(bigEndianStr('M', '5', '6', '6')); }
        inline bool isM567() const { return isOpcode(bigEndianStr('M', '5', '6', '7')); }
        inline bool isM568() const { return isOpcode(bigEndianStr('M', '5', '6', '8')); }
        inline bool isM569() const { return isOpcode(bigEndianStr('M', '5', '6', '9')); }
        inline bool isM665() const { return isOpcode(bigEndianStr('M', '6', '6', '5')); }
        inline bool isM906() const { return isOpcode(bigEndianStr('M', '9', '0', '6')); }
        inline bool isM998() const { return isOpcode(bigEndianStr('M', '9', '9', '8')); }
        inline bool isM999() const { return isOpcode(bigEndianStr('M', '9', '9', '9')); }
        inline bool isTxxx() const { return isFirstChar('T'); }
    private:
        //Make the letter passed uppercase if it was not before.
        //Must be done because gcode is case-insensitive (G1 == g1)
        inline char upper(char letter) const {
            if (letter >= 'a' && letter <= 'z') { //if lowercase
                letter += ('A' - 'a'); //add the offset between uppercase and lowercase letters for our character set.
            }
            return letter;
        }
        inline void setArgument(char letter, float value) {
            letter = upper(letter);
            int index = letter - 'A';
            this->arguments[index] = value;
        }
        inline bool isOpcode(uint32_t op) const {
            return opcodeStr == op;
        }
        inline float normalizeDutyCycle(float duty) const {
            if (duty > 1) {
                //host thinks we're working from 0 to 255
                return duty / 255.0f;
            }
            return duty;
        }
        bool isFirstChar(char c) const;
};

}
#endif
