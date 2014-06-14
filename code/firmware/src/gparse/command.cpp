#include "command.h"

namespace gparse {

Command::Command(std::string const& cmd) {
	//initialize the command from a line of GCode
	std::string piece;
	for (char chr : cmd) { //split the command on spaces.
		if (chr == ' ' || chr == '\n' || chr == '\t' || chr == '*') {
			if (piece.length()) { //allow for multiple spaces between parameters
				if (piece[0] != 'N') { //don't store optional line numbers.
					this->pieces.push_back(piece);
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
		this->pieces.push_back(piece);
	}
}

std::string Command::getOpcode() const {
	return this->pieces[0];
}

std::string Command::toGCode() const {
	std::string r="";
	for (std::string const& s : this->pieces) {
		if (r.length()) {
			r += ' ';
		}
		r += s;
	}
	return r + '\n';
}

}
