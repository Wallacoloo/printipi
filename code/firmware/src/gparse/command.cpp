#include "command.h"

Command::Command(std::string &cmd) {
	//initialize the command from a line of GCode
	std::string piece;
	for (auto chr : cmd) { //split the command on spaces.
		if (chr == ' ') {
			this->pieces.push_back(piece);
			piece = "";
		} else {
			piece += chr;
		}
	}
	if (piece.length()) {
		this->pieces.push_back(piece);
	}
}

std::string Command::toGCode() {
	std::string r="";
	for (std::string &s : this->pieces) {
		if (r.length()) {
			r += ' ';
		}
		r += s;
	}
	return r + '\n';
}
