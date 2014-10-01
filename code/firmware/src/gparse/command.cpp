#include "command.h"

namespace gparse {


Command::Command(std::string const& cmd) : opcodeStr(0), arguments({NAN}) {
    //possible GCodes to handle:
    //N123 M105*nn
    //G1 X5.2 Y-3.72
    //G82 X Y
    // [empty]
    //G1 ;LALALA
    //;^_^;
    //initialize the command from a line of GCode
    std::string::const_iterator argumentStart; //iterator pointing to the first character in the current argument being parsed.
    std::string::const_iterator it=cmd.begin();
    for(; it != cmd.end() && (*it == ' ' || *it == '\t'); ++it) {} //skip leading spaces
    if (cmd[0] == 'N') { //line-number
        do {
            ++it;
        } while (it != cmd.end() && *it != ' ' && *it != '\n' && *it != '\t' && *it != '*' && *it != ';');
        for(; it != cmd.end() && (*it == ' ' || *it == '\t'); ++it) {} //skip spaces between line-number and opcode.
    }
    //now at the first character of the opcode
    for (; it != cmd.end() && *it != ' ' && *it != '\n' && *it != '\t' && *it != '*' && *it != ';'; ++it) {
        opcodeStr = (opcodeStr << 8) + *it; //TODO: case insensitivity (only first char must be case-insensitive; rest are numbers)
    }
    while (true) {
        //now at the first space after opcode or end of cmd or at the '*' character of checksum.
        for (; it != cmd.end() && (*it == ' ' || *it != '\t'); ++it) { //skip spaces
        }
        if (it == cmd.end() || *it == '*' || *it == ';' || *it == '\n') { //exit if end of line
            return;
        }
        //now at a LETTER, assuming valid command.
        char param = *it++;
        //now at either the end, space, * or ; OR a number.
        float value = 0;
        if (it != cmd.end() && *it != ' ' && *it != '\t' && *it == '\n' && *it != '*' && *it != ';') { 
            //Now we are at the first character of a number.
            //How to parse a float? Can use atof, strtof, or sscanf.
            //atof is basic, and won't tell how many characters we must advance
            //strtof will skip whitespace (which is invalid), and tells us how many chars to advance
            //sscanf is overly heavy, but won't tell how many characters we must advance
            //ALL THE ABOVE C-FUNCTIONS WORK WITH NULL-TERMINATED STRINGS.
            //Also, atof, etc, use the locale (so decimal point may be ',', not '.'.
            //stof can work with c++ strings (no offset), and reports # of chars parsed
            // '.' separator is the only valid one for gcode (source: http://git.geda-project.org/pcb/commit/?id=6f422eeb5c6a0e0e541b20bfc70fa39a8a2b5af1)
            char *afterVal;
            const char *cmdCStr = cmd.c_str();
            value = strtof(cmdCStr + (it-cmd.begin()), &afterVal); //read a float and set afterVal to point 
            it += (afterVal-cmdCStr); //advance iterator to past the number.
        }
        setArgument(param, value);
        //now at either space, *, ;, or end.
    }
    
    //spaces should be handled below.
    //Now split the command on spaces or tabs:
    /*for (; it != cmd.end() && *it != ';' && *it != '\n'; ++it) {
        char chr = *it;
        if (chr == ' ' || chr == '\t' || chr == '*') {
            if (piece.length()) { //allow for multiple spaces between parameters
                this->addPiece(piece);
                piece = "";
            }
            if (chr == '*') { //checksum. Don't verify for now.
                break;
            }
        } else {
            piece += chr;
        }
    }
    if (piece.length()) {
        this->addPiece(piece);
    }*/
}

/*void Command::addPiece(std::string const& piece) {
    this->pieces.push_back(piece);
}*/

bool Command::isFirstChar(char c) const {
            char s[4];
            //extract bytes from opcodeStr.
            //cannot just cast to char* due to endianness.
            s[0] = (char)((opcodeStr & 0xff000000u) >> 24);
            s[1] = (char)((opcodeStr & 0xff0000u) >> 16);
            s[2] = (char)((opcodeStr & 0xff00u) >> 8);
            s[3] = (char)(opcodeStr & 0xffu);
            return (s[0] == c || (s[0] == 0 && 
                     (s[1] == c || (s[1] == 0 &&
                       (s[2] == c || (s[2] == 0 && 
                         s[3] == c)
                       )
                     ))
                   ));
                       
        }

/*bool Command::empty() const {
    return this->opcode.empty();
}*/

std::string Command::getOpcode() const {
    //return opcode;
    std::string ret;
    char s[4];
    //extract bytes from opcodeStr.
    //cannot just cast to char* due to endianness.
    s[0] = (char)((opcodeStr & 0xff000000u) >> 24);
    s[1] = (char)((opcodeStr & 0xff0000u) >> 16);
    s[2] = (char)((opcodeStr & 0xff00u) >> 8);
    s[3] = (char)(opcodeStr & 0xffu);
    for (int i=0; i<4; ++i) {
        if (s[i]) { //add non-zero characters to the string
            ret += s[i];
        }
    }
    return ret;
}

std::string Command::toGCode() const {
    std::string r=getOpcode();
    /*for (std::string const& s : this->pieces) {
        r += ' ';
        r += s;
    }*/
    for (char c='A'; c<='Z'; ++c) {
        if (hasParam(c)) {
            r += ' ';
            r += c;
            r += std::to_string(getFloatParam(c));
        }
    }
    return r + '\n';
}

bool Command::hasParam(char label) const {
    /*for (const std::string &p : this->pieces) {
        if (p[0] == label) {
            return true;
        }
    }
    return false;*/
    return getFloatParam(label) != GPARSE_ARG_NOT_PRESENT;
}

/*std::string Command::getStrParam(char label, bool &hasParam) const {
    for (const std::string &p : this->pieces) {
        if (p[0] == label) {
            if (p[1] == ':') {
                hasParam = true;
                return p.substr(2);
            } else {
                hasParam = true;
                return p.substr(1);
            }
        }
    }
    hasParam = false;
    return "";
}*/
/*std::string Command::getStrParam(char label) const {
    bool _ignore;
    return this->getStrParam(label, _ignore);
}*/
float Command::getFloatParam(char label, float def, bool &hasParam) const {
    float val = arguments[upper(label)-'A'];
    hasParam = (val != GPARSE_ARG_NOT_PRESENT);
    if (!hasParam) {
        val = def;
    }
    return val;
    //std::string s = this->getStrParam(label, hasParam);
    //return hasParam ? std::stof(s) : def;
}

}
