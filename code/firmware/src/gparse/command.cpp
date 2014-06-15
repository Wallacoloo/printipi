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
float Command::getFloatParam(char label) const {
	return std::stof(this->getStrParam(label));
}

float Command::getX() const {
	return this->getFloatParam('X');
}
float Command::getY() const {
	return this->getFloatParam('Y');
}
float Command::getE() const {
	return this->getFloatParam('E');
}

}
