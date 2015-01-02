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

/* 
 * gparse/com.h
 *
 * Com manages the low-level interfacing with whatever is controlling this printer.
 * reads are non-blocking, so tendCom() must be called on a regular basis.
 * once tendCom returns true, then a command is available via getCommand(), and a reply can be sent to the host via reply(...)
 *
 * Communication is typically done over a serial interface, but Com accepts any file descriptor,
 *   so communication can be done via stdin (/dev/stdin), or perhaps commands can be directly fed from a gcode file (untested).
 */
 

#ifndef GPARSE_COM_H
#define GPARSE_COM_H

#include <string>
#include "command.h"
#include "response.h"

#define NO_HANDLE -1 //null file descriptor


namespace gparse {

class Com {
    int _readFd;
    int _writeFd;
    //store any partially-received line that hasn't been fully parsed
    std::string _pending;
    //The last parsed command that is awaiting a reply
    Command _parsed;
    //Most of the time, the files being read from are actually streams of some sort, and so an EOF just means the data isn't yet ready.
    //But when reading from an ACTUAL file, an EOF actually does indicate the end of commands.
    bool _dieOnEof;
    bool _isAtEof;
    public:
        static const std::string NULL_FILE_STR;
    public:
        Com();

        //set @dieOnEof=true when reading from an actual, fix-length file, instead of a stream.
        //useful when dealing with "subprograms" (printing from a file), in which the replies don't need to be sent back to the main com channel.
        Com(const std::string &fileR, const std::string &fileW=NULL_FILE_STR, bool dieOnEof=false);

        //returns true if there is a command ready to be interpreted.
        bool tendCom();

        bool hasReadFile() const;
        bool hasWriteFile() const;

        //if reading with dieOnEof=true, and the last command has been parsed (but not necessarily responded to),
        //  then this function will return true
        bool isAtEof() const;

        //returns any pending command.
        //
        //sequential calls to getCommand() will all return the same command, until reply() is called, at which point the next command will be parsed.
        const Command& getCommand() const;
        
        //void reply(const std::string &resp);
        void reply(const Response &resp);
        
            
};

};
#endif
