/* 
 * Printipi/main.cpp
 * (c) 2014 Colin Wallace
 *
 * main is the program's main entry point.
 * main handles command line arguments, and instantiates the serial communications, State, and machine-specific driver.
 * Driver type is passed into this compilation unit as a command-line argument (gcc -DMACHINE=...)
 *
 * Printipi discussions:
 *   http://forums.reprap.org/read.php?2,396157
 *   https://groups.google.com/forum/#!searchin/deltabot/wallacoloo|sort:relevance/deltabot/JQNpmnlYYUc/_6V6SYcOGMUJ
 *   http://youtube.com/watch?v=g4UD5MRas3E
 *   (referenced) http://3dprintboard.com/showthread.php?5121-MOD-t-may-make-3D-printing-commonplace
 */
 
//Note: this file is great for debugging with gdb: https://gist.githubusercontent.com/skyscribe/3978082/raw/e8a0c8daec409e24b29f7c14cf74140a43a9278c/.gdbinit


#define COMPILING_MAIN //used elsewhere to do only one-time warnings, etc.
#include "common/typesettings.h" //check types
#include <string>
#include <sys/mman.h> //for mlockall
#include "common/logging.h"

#include "gparse/com.h"
#include "state.h"
#include "argparse.h"

//MACHINE_PATH is calculated in the Makefile and then passed as a define through the make system (ie gcc -DMACHINEPATH='"path"')
//To set the path, call make MACHINE_PATH=...
//or, call make MACHINE=<machine>, eg MACHINE=Kossel (case-sensitive) and the path will be calculated from that (drivers/machines/kossel.h)
#include MACHINE_PATH

void printUsage(char* cmd) {
    //#ifndef NO_USAGE_INFO
    LOGE("usage: %s [ttyFile] [--help] [--quiet] [--verbose]\n", cmd);
    //std::cerr << "usage: " << cmd << " ttyFile" << std::endl;
    //#endif
    //exit(1);
}

int main_(int argc, char** argv) {
    char defaultSerialFile[] = "/dev/stdin";
    char defaultOutFile[] = "/dev/null";
    char* serialFileName; //file which Com reads from
    char* outFile = defaultOutFile; //file which Com posts responses 
    SchedulerBase::configureExitHandlers(); //useful to do this first-thing for catching debug info.
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
    if (argc < 2 || argv[1][0] == '-') { //if no arguments, or if first argument (and therefore all args) is an option
        //printUsage(argv[0]);
        serialFileName = defaultSerialFile;
    } else {
        serialFileName = argv[1];
        if (argc >2 && argv[2][0] != '-') { //second argument is for the output file
            outFile = argv[2];
        }
    }
    
    //prevent page-swaps to increase performace:
    int retval = mlockall(MCL_FUTURE|MCL_CURRENT);
    if (retval) {
        LOGW("Warning: mlockall (prevent memory swaps) in main.cpp::main() returned non-zero: %i\n", retval);
    }
    
    //Open the serial device:
    LOG("Serial file: %s\n", serialFileName);
    gparse::Com com = gparse::Com(std::string(serialFileName), std::string(outFile));
    
    //instantiate main driver:
    typedef drv::MACHINE MachineT;
    MachineT driver;
    State<MachineT> state(driver, com);
    
    state.eventLoop();
    return 0;
}

int main(int argc, char** argv) {
    try { //wrap in a try/catch loop so we can safely clean up (disable IOs)
        return main_(argc, argv);
    } catch (const std::exception *e) {
        LOGE("caught std::exception*: %s. ... Exiting\n", e->what());
        return 1;
    } catch (const std::exception &e) {
        LOGE("caught std::exception&: %s. ... Exiting\n", e.what());
        return 1;
    } catch (...) {
        LOGE("caught unknown exception. Exiting\n");
        return 1;
    }
}
