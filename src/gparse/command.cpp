#include "command.h"

namespace gparse {


Command::Command(std::string const& cmd) : opcodeStr(0) {
    arguments.fill(GPARSE_ARG_NOT_PRESENT); //initialize all arguments to default value
    //possible GCodes to handle:
    //N123 M105*nn
    //G1 X5.2 Y-3.72
    //G82 X Y
    // [empty]
    //G1 ;LALALA
    //;^_^;
    //initialize the command from a line of GCode
    //iterator pointing to the first character in the current argument being parsed.
    std::string::const_iterator argumentStart; 
    std::string::const_iterator it=cmd.begin();

    //skip leading spaces
    for(; it != cmd.end() && (*it == ' ' || *it == '\t'); ++it) {} 
    //Check for a line-number
    if (cmd[0] == 'N' || cmd[0] == 'n') {
        do {
            ++it;
        } while (it != cmd.end() && *it != ' ' && *it != '\n' && *it != '\t' && *it != '*' && *it != ';');
        //skip spaces between line-number and opcode.
        for(; it != cmd.end() && (*it == ' ' || *it == '\t'); ++it) {} 
    }

    //now at the first character of the opcode
    for (; it != cmd.end() && *it != ' ' && *it != '\n' && *it != '\t' && *it != '*' && *it != ';'; ++it) {
        opcodeStr = (opcodeStr << 8) + upper(*it); //Note: only the first really character needs to be 'upper'd
    }
    while (true) {
        //now at the first space after opcode or end of cmd or at the '*' character of checksum.
        for (; it != cmd.end() && (*it == ' ' || *it == '\t'); ++it) { //skip spaces
        }
        if (it == cmd.end() || *it == '*' || *it == ';' || *it == '\n') { //exit if end of line
            return;
        }
        //now at a LETTER, assuming valid command.
        char param = *it++;
        //now at either the end, space, * or ; OR a number OR a slash (/) if a filename
        //if (param == '/') { //filename; have to parse as a string.
        if (isM117() || isM32()) {
            //Some whackjob decided that M117 and M32 were special enough to require an entirely different parameter parsing routine,
            // and we are forced to do their bidding here.
            // God save us if we ever want to add additional parameters to either of these m-codes
            std::string::const_iterator first = it-1;
            //advance to the end of the string parameter:
            for (; it != cmd.end() /*&& *it != ' ' && *it != '\t' */ && *it != '\n' && *it != '*' && *it != ';'; ++it) {}
            //Probably a good idea to trim trailing whitespace
            std::string::const_iterator lastCharToInclude = it;
            do { --lastCharToInclude; } while(*lastCharToInclude == ' ' || *lastCharToInclude == '\t');
            //now the iterator will point to the last character which we want to include as part of the parameter
            //get the string parameter as a substr (likely more efficient than just appending the characters one-by-one):
            this->specialStringParam = cmd.substr(first - cmd.begin(), lastCharToInclude+1-first);
        } else {
            float value = 0;
            if (it != cmd.end() && *it != ' ' && *it != '\t' && *it != '\n' && *it != '*' && *it != ';') { 
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
                const char *floatStart = cmdCStr + (it-cmd.begin());
                //read a float and set afterVal to point to the first character (or null-terminator) after the float
                value = strtof(floatStart, &afterVal); 
                //advance iterator to past the number.
                it += (afterVal-floatStart); 
            }
            setArgument(param, value);
        }
        //now at either space, *, ;, or end.
    }
}

bool Command::isFirstChar(char c) const {
    //Check if the first character of the opcode is `c'
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
    for (char c='A'; c<='Z'; ++c) {
        if (hasParam(c)) {
            r += ' ';
            r += c;
            r += std::to_string(getFloatParam(c));
        }
    }
    if (!getSpecialStringParam().empty()) {
        r += ' ';
        r += getSpecialStringParam();
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
    //return arguments[upper(label)-'A'] != GPARSE_ARG_NOT_PRESENT; //BREAKS FOR NAN
    bool a = arguments[upper(label)-'A'] != GPARSE_ARG_NOT_PRESENT;
    bool b = (!std::isnan(GPARSE_ARG_NOT_PRESENT) || !std::isnan(arguments[upper(label)-'A']));
    return a && b;
}

/*float Command::getFloatParam(char label) const {
    return arguments[upper(label)-'A'];
}*/

float Command::getFloatParam(char label, float def, bool &hasParam) const {
    hasParam = this->hasParam(label);
    return hasParam ? arguments[upper(label)-'A'] : def;
    //std::string s = this->getStrParam(label, hasParam);
    //return hasParam ? std::stof(s) : def;
}

}
