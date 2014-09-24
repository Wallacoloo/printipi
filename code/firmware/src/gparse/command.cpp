#include "command.h"

namespace gparse {


Command::Command(std::string const& cmd) : opcodeStr(0) {
    //initialize the command from a line of GCode
    std::string piece;
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
        opcodeStr = (opcodeStr << 8) + *it;
    }
    //now at the first space after opcode or end of cmd or at the '*' character of checksum.
    //spaces should be handled below.
    //Now split the command on spaces or tabs:
    for (; it != cmd.end() && *it != ';' && *it != '\n'; ++it) {
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
    }
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
    for (std::string const& s : this->pieces) {
        r += ' ';
        r += s;
    }
    return r + '\n';
}

bool Command::hasParam(char label) const {
    for (const std::string &p : this->pieces) {
        if (p[0] == label) {
            return true;
        }
    }
    return false;
}

std::string Command::getStrParam(char label, bool &hasParam) const {
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
}
/*std::string Command::getStrParam(char label) const {
    bool _ignore;
    return this->getStrParam(label, _ignore);
}*/
float Command::getFloatParam(char label, float def, bool &hasParam) const {
    std::string s = this->getStrParam(label, hasParam);
    return hasParam ? std::stof(s) : def;
}
/*float Command::getFloatParam(char label, float def) const {
    bool _ignore;
    return this->getFloatParam(label, def, _ignore);
}
float Command::getFloatParam(char label, bool &hasParam) const {
    return this->getFloatParam(label, NAN, hasParam);
}*/

/*float Command::getX(float def) const {
    return this->getFloatParam('X', def);
}
float Command::getX(bool &hasParam) const {
    return this->getFloatParam('X', hasParam);
}
float Command::getY(float def) const {
    return this->getFloatParam('Y', def);
}
float Command::getY(bool &hasParam) const {
    return this->getFloatParam('Y', hasParam);
}
float Command::getZ(float def) const {
    return this->getFloatParam('Z', def);
}
float Command::getZ(bool &hasParam) const {
    return this->getFloatParam('Z', hasParam);
}
float Command::getE(float def) const {
    return this->getFloatParam('E', def);
}
float Command::getE(bool &hasParam) const {
    return this->getFloatParam('E', hasParam);
}
float Command::getF(float def) const {
    return this->getFloatParam('F', def);
}
float Command::getF(bool &hasParam) const {
    return this->getFloatParam('F', hasParam);
}
float Command::getS(float def) const {
    return this->getFloatParam('S', def);
}
float Command::getS(bool &hasParam) const {
    return this->getFloatParam('S', hasParam);
}*/

/*bool Command::hasX() const {
    return hasParam('X');
}
bool Command::hasY() const {
    return hasParam('Y');
}
bool Command::hasZ() const {
    return hasParam('Z');
}
bool Command::hasE() const {
    return hasParam('E');
}
bool Command::hasF() const {
    return hasParam('F');
}
bool Command::hasS() const {
    return hasParam('S');
}

bool Command::hasAnyXYZParam() const {
    return hasX() || hasY() || hasZ();
}
bool Command::hasAnyXYZEParam() const {
    return hasAnyXYZParam() || hasE();
}*/

}
