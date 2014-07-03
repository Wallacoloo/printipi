#include "command.h"

namespace gparse {

const Command Command::OK("ok");

Command::Command(std::string const& cmd) {
	//initialize the command from a line of GCode
	std::string piece;
	std::string::const_iterator it=cmd.begin();
	if (cmd[0] == 'N') { //line-number
		do {
		    ++it;
		} while (it != cmd.end() && *it != ' ' && *it != '\n' && *it != '\t' && *it != '*');
	}
	for (; it != cmd.end(); ++it) { //split the command on spaces...
		char chr = *it;
		if (chr == ' ' || chr == '\n' || chr == '\t' || chr == '*') {
			if (piece.length()) { //allow for multiple spaces between parameters
				this->addPieceOrOpcode(piece);
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
		this->addPieceOrOpcode(piece);
		//this->pieces.push_back(piece);
	}
}

void Command::addPieceOrOpcode(std::string const& piece) {
	if (this->getOpcode().length()) {
		this->pieces.push_back(piece);
	} else {
		this->opcode = piece;
	}
}

std::string Command::getOpcode() const {
	return this->opcode;
}

std::string Command::toGCode() const {
	std::string r=opcode;
	for (std::string const& s : this->pieces) {
		r += ' ';
		r += s;
	}
	return r + '\n';
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
std::string Command::getStrParam(char label) const {
    bool _ignore;
    return this->getStrParam(label, _ignore);
}
float Command::getFloatParam(char label, float def, bool &hasParam) const {
    std::string s = this->getStrParam(label, hasParam);
    return hasParam ? std::stof(s) : def;
}
float Command::getFloatParam(char label, float def) const {
    bool _ignore;
    return this->getFloatParam(label, def, _ignore);
}
float Command::getFloatParam(char label, bool &hasParam) const {
    return this->getFloatParam(label, NAN, hasParam);
}

float Command::getX(float def) const {
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

}
