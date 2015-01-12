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
 * Printipi/main.cpp
 *
 * main is the program's main entry point.
 * main handles command line arguments, and instantiates the serial communications, State, and machine-specific driver.
 * Driver type is passed into this compilation unit as a command-line argument (gcc -DMACHINE=...)
 *
 * Printipi Github repository: https://github.com/Wallacoloo/printipi/
 * Printipi Google Group (forum): https://groups.google.com/forum/#!forum/printipi
 *
 * Miscellaneous Printipi discussions:
 *   http://forums.reprap.org/read.php?2,396157
 *   https://groups.google.com/forum/#!searchin/deltabot/wallacoloo|sort:relevance/deltabot/JQNpmnlYYUc/_6V6SYcOGMUJ
 *   http://youtube.com/watch?v=g4UD5MRas3E
 *   http://youtube.com/watch?v=gAruwqOEuPs
 */

#define COMPILING_MAIN //used elsewhere to do one-time warnings, etc.

#include "compileflags.h"

//Includes for the CATCH testing framework
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

#include <string>
#include <sys/mman.h> //for mlockall
#include <iostream> //for std::cin
#include "common/logging.h"

#include "gparse/com.h"
#include "state.h"
#include "argparse.h"
#include "filesystem.h"

//MACHINE_PATH is calculated in the Makefile and then passed as a define through the make system (ie gcc -DMACHINEPATH='"path"')
//To set the path, call make MACHINE_PATH=...
//or, call make MACHINE=<machine>, eg MACHINE=rpi::KosselPi (case-sensitive) and the path will be calculated from that (src/machines/rpi/kossel.h)
#include MACHINE_PATH


#define STRINGIFY(x) #x

static void printUsage(char* cmd) {
    //#ifndef NO_USAGE_INFO
    LOGE("usage: %s [input file=/dev/stdin] [output file=/dev/null] [--help] [--quiet] [--verbose]\n", cmd);
    LOGE("examples:\n");
    LOGE("  print a gcode file: %s file.gcode\n", cmd);
    LOGE("  mock serial port: %s /dev/tty3dpm /dev/tty3dps\n", cmd);
}

int testmain(int argc, char **argv) {
    //if the program was compiled in test mode, then run the unit tests and exit
    #if DO_TESTS
        int result = Catch::Session().run(argc, argv);
        return result;
    #else
        (void)argc; (void)argv; //unused
    #endif
    return 0;
}

int main_(int argc, char **argv) {
    //useful to setup fail-safe exit routines first-thing for catching debug info.
    SchedulerBase::configureExitHandlers(); 
    
    if (argparse::cmdOptionExists(argv, argv+argc, "--quiet")) {
        logging::disable();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "--debug")) {
        logging::enableDebug();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "--verbose")) {
        logging::enableVerbose();
    }
    if (argparse::cmdOptionExists(argv, argv+argc, "-h") || argparse::cmdOptionExists(argv, argv+argc, "--help")) {
        printUsage(argv[0]);
        return 0;
    } 
    LOG("Printipi: built for machine: '" STRINGIFY(MACHINE) "'\n");
    
    char* fsRootArg = argparse::getCmdOption(argv, argv+argc, "--fsroot");
    std::string fsRoot = fsRootArg ? std::string(fsRootArg) : "/";
    LOG("Filesystem root: %s\n", fsRoot.c_str());
    FileSystem fs(fsRoot);
    
    gparse::Com com;
    //if input is stdin, or a two-way pipe, then it likely means we want to keep that channel open forever
    //  whereas if it's a gcode file, then calls to M32 (print from file) should pause the original input file
    bool keepPersistentCom = false;

    //if no arguments, or if first argument (and therefore all args) is an option, 
    //  then take gcode commands from stdin
    if (argc < 2 || argv[1][0] == '-') { 
        //std::cin does not allow to check if there are characters to be read, whereas /dev/stdin DOES.
        //  we need nonblocking I/O, so this is crucial.
        //com = std::move(gparse::Com(gparse::Com::shareOwnership(&std::cin)));
        com = std::move(gparse::Com("/dev/stdin"));
        //reading from stdin; want to keep that channel active even when printing from file, etc.
        keepPersistentCom = true;
    } else {
        //otherwise, first parameter must be a filename from which to read gcode commands
        //second argument is for the output file
        if (argc > 2 && argv[2][0] != '-') { 
            com = std::move(gparse::Com(std::string(argv[1]), std::string(argv[2])));
            //hints at a dual-way pipe (host <-> firmware communication); preserve communiation channel to host
            keepPersistentCom = true;
        } else {
            //no second file; just read from the supplied input file
            com = std::move(gparse::Com(std::string(argv[1])));
        }
    }
    
    //prevent page-swaps to increase performace:
    int retval = mlockall(MCL_FUTURE|MCL_CURRENT);
    if (retval) {
        LOGW("Warning: mlockall (prevent memory swaps) in main.cpp::main() returned non-zero: %i\n", retval);
    }
        
    State<machines::MACHINE> state(machines::MACHINE(), fs, keepPersistentCom);
    state.addComChannel(std::move(com));
    state.eventLoop();
    return 0;
}

int main(int argc, char** argv) {
    if (DO_TESTS) {
        return testmain(argc, argv);
    } else {
        try { //wrap in a try/catch loop so we can safely clean up (disable IOs)
            return main_(argc, argv);
        } catch (const std::exception *e) {
            LOGE("caught std::exception*: %s. ... Exiting\n", e->what());
            #if CLEAN_EXIT
                return 1; //don't rethrow exceptions; return an error code instead
            #endif
            throw;
        } catch (const std::exception &e) {
            LOGE("caught std::exception&: %s. ... Exiting\n", e.what());
            #if CLEAN_EXIT
                return 1; //don't rethrow exceptions; return an error code instead
            #endif
            throw;
        } catch (...) {
            LOGE("caught unknown exception. Exiting\n");
            #if CLEAN_EXIT
                return 1; //don't rethrow exceptions; return an error code instead
            #endif
            throw;
        }
    }
}

