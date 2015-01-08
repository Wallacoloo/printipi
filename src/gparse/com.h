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

#ifndef GPARSE_COM_H
#define GPARSE_COM_H

#include <string>
#include <fstream>
#include <memory> //for std::unique_ptr
#include "command.h"
#include "response.h"

namespace gparse {

/* 
 * Com manages the low-level interfacing with whatever is controlling this printer.
 * reads are non-blocking, so tendCom() must be called on a regular basis.
 * once tendCom returns true, then a command is available via getCommand(), and a reply can be sent to the host via reply(...)
 *
 * Communication is typically done over a serial interface, but Com accepts any file descriptor,
 *   so communication can be done via stdin (/dev/stdin) or commands can be directly fed from a gcode file.
 */
class Com {
    //Use a custom deleter with std::unique_ptr that allows us to indicate whether we actually "own" the pointer,
    //  or are just "borrowing" it.
    class ComStreamDeleter {
        bool hasOwnership;
        public:
            ComStreamDeleter(bool claimOwnership=true) : hasOwnership(claimOwnership) {}
            template <class T> void operator()(T* ptr) {
                if (hasOwnership) {
                    delete ptr;
                }
            }
    };

    //Used during Com construction to wrap the stream input to give an indication of who owns the stream.
    //Instead of using directly, it's a better idea to refer to the public <shareOwnership> and <giveFullOwnership> functions
    template <typename T> class ComStreamOwnershipMarker {
        friend class Com;
        T argument;
        bool hasOwnership;
        public:
            ComStreamOwnershipMarker(const T &argument, bool hasOwnership) : argument(argument), hasOwnership(hasOwnership) {}
            //allow implicit casts to compatible argument types.
            //  useful when e.g. trying to instantiate a Base* from a Derived*
            template <typename TBase> operator ComStreamOwnershipMarker<TBase>() const {
                return ComStreamOwnershipMarker<TBase>(argument, hasOwnership);
            }
    };
    //Have to use unique_ptrs because fstreams aren't movable for gcc < 5.0
    std::unique_ptr<std::istream, ComStreamDeleter> _readFd;
    std::unique_ptr<std::ostream, ComStreamDeleter> _writeFd;
    //store any partially-received line that hasn't been fully parsed
    std::string _pending;
    //The last parsed command that is awaiting a reply
    Command _parsed;
    //Most of the time, the files being read from are actually streams of some sort, and so an EOF just means the data isn't yet ready.
    //But when reading from an ACTUAL file, an EOF actually does indicate the end of commands.
    bool _dieOnEof;
    bool _isAtEof;
    public:
        //Whenever you pass a file pointer to the Com constructor, you must explicitly mark who the owner should be.
        //If you wish for the caller to retain ownership, call Com(..., shareOwnership(file), ...)
        template <typename T> static ComStreamOwnershipMarker<T> shareOwnership(const T &stream) {
            return ComStreamOwnershipMarker<T>(stream, false);
        }
        //Whenever you pass a file pointer to the Com constructor, you must explicitly mark who the owner should be.
        //If you wish to pass ownership over to Com, call Com(..., giveFullOwnership(file), ...)
        template <typename T> static ComStreamOwnershipMarker<T> giveFullOwnership(const T &stream) {
            return ComStreamOwnershipMarker<T>(stream, true);
        }
        //If the Com channel shouldn't have an input stream, then pass NO_INPUT_STREAM() as the first parameter to the constructor
        static ComStreamOwnershipMarker<std::istream*> NO_INPUT_STREAM() {
            return giveFullOwnership<std::istream*>(nullptr);
        }
        //If the Com channel shouldn't have an output stream, then pass NO_OUTPUT_STREAM() as the first parameter to the constructor
        static ComStreamOwnershipMarker<std::ostream*> NO_OUTPUT_STREAM() {
            return giveFullOwnership<std::ostream*>(nullptr);
        }
    public:
        //set @dieOnEof=true when reading from an actual, fix-length file, instead of a stream.
        //useful when dealing with "subprograms" (printing from a file), in which the replies don't need to be sent back to the main com channel.
        //Com(const std::string &fileR=NULL_FILE_STR, const std::string &fileW=NULL_FILE_STR, bool dieOnEof=false);
        template <typename RType=ComStreamOwnershipMarker<std::istream*>, typename WType=ComStreamOwnershipMarker<std::ostream*> > 
          Com(const RType &readStream=NO_INPUT_STREAM(), const WType &writeStream=NO_OUTPUT_STREAM(), bool dieOnEof=false) 
          : _readFd(nullptr), 
            _writeFd(nullptr),
            _dieOnEof(dieOnEof),
            _isAtEof(false) {
                setInputFile(readStream);
                setOutputFile(writeStream);
        }

        //returns true if there is a command ready to be interpreted.
        bool tendCom();

        bool hasReadFile() const;
        bool hasWriteFile() const;

        //if reading with dieOnEof=true, and the last command has been parsed (but not necessarily responded to),
        //  then this function will return true
        bool isAtEof() const;
    private:
        //release any previous input file and replace it with the file designated by @name
        void setInputFile(const std::string &name);
        void setInputFile(ComStreamOwnershipMarker<std::istream*> stream);

        //release any previous output file and replace it with the file designated by @name
        void setOutputFile(const std::string &name);
        void setOutputFile(ComStreamOwnershipMarker<std::ostream*> stream);
    public:
        //returns any pending command.
        //
        //sequential calls to getCommand() will all return the same command, until reply() is called, at which point the next command will be parsed.
        const Command& getCommand() const;
        
        void reply(const Response &resp);
        
            
};

};
#endif
