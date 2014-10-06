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

/*TODO:
 *  What is ReplicatorG software? https://github.com/makerbot/ReplicatorG
 *    It appears that it MAY be similar to this project, although it uses Java, so perhaps it isn't designed to be run as firmware.
 *    Maybe it sends a dumb control signal to the electronics, parsing the G-code into simpler commands?
 *  What is SmoothieWare? https://github.com/Smoothieware/Smoothieware
 *    It claims to be aimed toward ARM cortex M3, and very active development
 *  Grbl appears to run (partially?) on the raspberry pi: https://github.com/grbl/grbl/issues/252
 *
 *  Investigate fixed-point math.
 *    http://pithesiser.wordpress.com/2013/02/17/on-raspberry-pi-performance/
 *      Indicates it MAY be faster, could be the same (esp if 64-bit)
 *
 *  Consider names: piprint (taken), printchef (exists), Rasprintian (play on raspbian), Rasprint, Rasprinti, Printipi, Piface (taken)
 *
 * *Run valgrind to hunt for uninitialized variables
 * *Prevent calling exit() from within an atexit handler
 * *Prevent stepping when endstop is triggered.
 *    NO: this is achieved better by bounding the axis/coordinates.
 *      The only use for this is really when homing, to allow buffering.
 * *Add enable pin
 * *Account for PWM pins when setting Scheduler queue size.
 *  Add (configurable) absolute PWM limits to hotend, etc.
 * *Make CoordMapT, etc return tuples instead of using writebacks
 * *NO: Reset I part of PID control when target changes
 *    instead, put a maximum limit on I.
 * *Allow M105, etc to instantly return the temperature
 *    Sort-of accomplished - a fresh command can be parsed while still waiting for the previous one to be completed.
 * *Drop temperature readings when thread has been interrupted.
 *    Best done by checking time since last call, due to future merging of sched thread with serial reading
 * *Add ability to put steppers to rest upon idle
 * *Make scheduler more resistant to skipping (make it so it can only run at up to, say, 2x speed to catch up. This minimizes missed steps)
 *  Improve SchedAdjuster formulae.
 *  Properly implement Scheduler::lastSchedTime for the case where the queue is empty
 * *Limit movement speed based on maximum extrusion rate.
 *  Look into coordinate rounding for State::queueMovement
 *  Document
 * *Put copywrite at head of every file.
 * *NO: Replace certain template parameters (eg STEPS_MM_1000 with std::ratio)
 *    Not worth it; will make code more difficult to read.
 * *Add way to define which driver to use without modifying code.
 * *Migrate to std::chrono::high_resolution_clock instead of timespec (http://www.cplusplus.com/reference/chrono/high_resolution_clock/)
 *    the clock types aren't defined in gcc-4.6.3, and, while defined in gcc-4.7, they behave incorrectly on arm!
 *    and/or use the Pi's timer directly (performance) http://mindplusplus.wordpress.com/2013/05/21/accessing-the-raspberry-pis-1mhz-timer/
 *      Since we have IO access, may as well use bcm2835_st_read() for faster clocking.
 *  *use std::chrono::steady_clock in the case that we aren't on Linux (else ChronoClockPosix)
 *    Perhaps make it configurable, eg as in http://stackoverflow.com/a/11485388/216292 (use std chrono, and also implement that interface for the pi)
 * *Look into using DMA for more precise and accurate scheduling (see info in hotend_control.txt)
 * *NO: Look into putting the scheduler (or entire program) into a kernel module
 *    Dma will get more precise timings than even a kernel module. Although technically, DMA in a kernel module would be safer than userland dma (fewer channel conflicts)
 *  Optimize gcode parser.
 *   *opcodes are conveniently 4 bytes (eg M123). Can fit in one int for direct comparisons, instead of string comparisons.
 *    Can also use one single string for entire command and array of char* to mark the delimiters.
 *    Could consider using a parser generator (yacc, bison, etc)
 *    Smoothie has a separate gcode parser: https://github.com/Smoothieware/Smoothieware/tree/edge/src/modules/communication/utils
 *      though it's not much better than this one.
 * *Make gcode parser handle empty lines and comments
 *  rename IODriver::stepForward/backward
 *  Add ability to configure the power-on coordinates, or default to homing before any commands.
 *  Optimize PID values
 * *Fix short-circuit operators in onIdleCpu
 *  Refactor the "enabler" system.
 *  Make IO Pin an interface (hardware abstraction), and then drivers can work with an IO pin across MANY different hardware systems, not just the Pi.
 * *Auto-find the hotend, rather than calling on the kossel driver.
 * *Move ioDrivers instantiation into the State.
 * *Prevent long sleeps in Scheduler when the next event is far off.
 * *Fix how homing changes the extruder coordinates.
 *    have CoordMap::getHomePosition return std::nan for E coordinate to indicate it has no home.
 * *Upgrade to larger cap for rcthermistor
 *  Make TempControl::_readInterval dynamic.
 *  Fix motion planner's baseTime.
 *    When it's a PWM event that's at the tail of the queue, the time is based on that instead of the last actual movement event.
 *    A better approach would be to save the last time returned from MotionPlanner, and ask the scheduler if that would occur in the future (good) or not (bad - offset the time).
 * *Optimize steppers
 *  Add a way to set a lower-bound on fan speed - either in the Machine driver, or the Fan driver itself.
 *    Also default fan speed for init.
 * *Add state to AcellerationProfiles
 *  make Sched::PwmInfo times use the same resolution as EventClockT (to avoid divisions by 1000 when offsetting events).
 * *Prevent rcthermistor from using 100% cpu when it errors.
 * *Fix relative extruder movements
 *  Fix low-pass filter
 * *Make NaN checks explicit
 * *Bound coordinates
 * *Make Endstop not be a static class
 * *Make EnableDisableDriver not be a static class
 *  Extruding should not force axis to be homed.
 *  Recognize when opening gcode file, and open as read-only (ie don't write 'ok' responses into it)
 *  Use the std:: file interface, rather than Linux file handlers.
 *  Have scheduler only schedule (ordered) output events, rather than preserving normal events too.
 *  Avoid accessing scheduler::interface.hardwareScheduler directly. Calls should be proxied by the interface itself.
 *  Speed up DMA init by caching physical addresses
*/

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
