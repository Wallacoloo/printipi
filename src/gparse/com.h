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
    std::string _pending;
    Command _parsed;
    public:
        Com();
        Com(const std::string &fileR);
        Com(const std::string &fileR, const std::string &fileW);
        //returns true if there is a command ready to be interpreted.
        bool tendCom();
        const Command& getCommand() const;
        //void reply(const Command &response);
        void reply(const std::string &resp);
        void reply(const Response &resp);
        inline bool hasReadFile() const {
            return _readFd != NO_HANDLE;
        }
        inline bool hasWriteFile() const {
            return _writeFd != NO_HANDLE;
        }
            
};

};
#endif
