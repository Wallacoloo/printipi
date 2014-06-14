#include "command.h"

Command::Command(std::string &cmd) {
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
