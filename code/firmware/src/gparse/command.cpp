#include "command.h"

namespace gparse {

const Command Command::OK("ok");

Command::Command(std::string const& cmd) {
	//initialize the command from a line of GCode
	std::string piece;
	for (char chr : cmd) { //split the command on spaces.
		if (chr == ' ' || chr == '\n' || chr == '\t' || chr == '*') {
			if (piece.length()) { //allow for multiple spaces between parameters
				if (piece[0] != 'N') { //don't store optional line numbers.
					this->addPieceOrOpcode(piece);
					//this->pieces.push_back(piece);
					piece = "";
				}
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

std::string Command::getStrParam(char label) const {
	for (const std::string &p : this->pieces) {
		if (p[0] == label) {
			if (p[1] == ':') {
				return p.substr(2);
			} else {
				return p.substr(1);
			}
		}
	}
	return "";
}
float Command::getFloatParam(char label, float def) const {
    std::string s = this->getStrParam(label);
    return s.empty() ? def : std::stof(s);
}

float Command::getX(float def) const {
	return this->getFloatParam('X', def);
}
float Command::getY(float def) const {
	return this->getFloatParam('Y', def);
}
float Command::getZ(float def) const {
	return this->getFloatParam('Z', def);
}
float Command::getE(float def) const {
	return this->getFloatParam('E', def);
}

}
