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

#ifndef TESTHELPER_H
#define TESTHELPER_H


#include "state.h"

#include <iostream>
#include <fstream> //for ifstream, ofstream
#include <thread>
#include <string>
#include <tuple>

#include "catch.hpp"
#include "filesystem.h"
#include "gparse/com.h"
//#include "common/logging.h"
#include "machines/machine.h"
#include "motion/accelerationprofile.h"
#include "motion/coordmap.h"

enum TestHelperCtorFlags {
    TESTHELPER_ENTER_EVENT_LOOP = 1,
    TESTHELPER_NO_ENTER_EVENT_LOOP = 0,

    TESTHELPER_PERSISTENT_ROOT_COM = 2,
    TESTHELPER_NO_PERSISTENT_ROOT_COM = 0,
};
//bitwise OR operator for TestHelperCtorFlags, to avoid a bitwise OR resulting in an integer type instead of a TestHelperCtorFlags type.
inline TestHelperCtorFlags operator|(TestHelperCtorFlags a, TestHelperCtorFlags b) {
    return static_cast<TestHelperCtorFlags>(static_cast<int>(a) | static_cast<int>(b));
}

struct DefaultGetIoDrivers {
    std::tuple<> operator()() const {
        return std::tuple<>();
    }
};

struct DefaultGetCoordMap {
    motion::CoordMap operator()() const {
        return motion::CoordMap();
    }
};

struct DefaultGetAccelerationProfile {
    motion::NoAcceleration operator()() const {
        return motion::NoAcceleration();
    }
};

template <typename GetIoDrivers=DefaultGetIoDrivers, typename GetCoordMap=DefaultGetCoordMap, typename GetAccelerationProfile=DefaultGetAccelerationProfile> class TestMachine : public machines::Machine {
    GetIoDrivers _getIoDrivers;
    GetCoordMap _getCoordMap;
    GetAccelerationProfile _getAccelerationProfile;
    public:
        TestMachine(const GetIoDrivers &getIoDrivers=GetIoDrivers(), const GetCoordMap &getCoordMap=GetCoordMap(), const GetAccelerationProfile &getAccelerationProfile=GetAccelerationProfile())
         : _getIoDrivers(getIoDrivers), _getCoordMap(getCoordMap), _getAccelerationProfile(getAccelerationProfile) {}
        auto getIoDrivers() const -> decltype(_getIoDrivers()) {
            return _getIoDrivers();
        }
        auto getCoordMap() const -> decltype(_getCoordMap()) {
            return _getCoordMap();
        }
        auto getAccelerationProfile() const -> decltype(_getAccelerationProfile()) {
            return _getAccelerationProfile();
        }

};

template <typename MachineT=machines::Machine> class TestHelper {
    //must be unique_ptr in order to be movable
    std::unique_ptr<std::ofstream> inputFile;
    std::unique_ptr<std::ifstream> outputFile;
    MachineT driver;
    FileSystem fs;
    State<MachineT> state;
    std::thread eventThread;
    public:
        TestHelper(const MachineT &machine=MachineT(), TestHelperCtorFlags flags=TESTHELPER_ENTER_EVENT_LOOP) 
        : inputFile([]() {
            //disable buffering before opening
            auto p = new std::ofstream();
            p->rdbuf()->pubsetbuf(0, 0);
            p->open("PRINTIPI_TEST_INPUT", std::fstream::out | std::fstream::trunc);
            return p;
        }()),
          outputFile([]() {
            //disable buffering before opening
            auto p = new std::ifstream();
            p->rdbuf()->pubsetbuf(0, 0);
            //must open with the ::out flag to automatically create the file if it doesn't exist
            p->open("PRINTIPI_TEST_OUTPUT", std::fstream::out | std::fstream::in | std::fstream::trunc);
            return p;
        }()),
          driver(machine), 
          fs("./"), 
          state(driver, fs, flags & TESTHELPER_PERSISTENT_ROOT_COM) {
            state.addComChannel(gparse::Com("PRINTIPI_TEST_INPUT", "PRINTIPI_TEST_OUTPUT"));
            //Note: the above inputFile and outputFile MUST be opened before com is instantiated, otherwise the files may not have been created
            //  and com will have a null file handle.   
            if (flags & TESTHELPER_ENTER_EVENT_LOOP) {
                this->threadedEventLoop();
            }      
        }
        TestHelper(TestHelper<MachineT> &&) = default;

        ~TestHelper() {
            exitOnce();
            //reset files before deleting their underlying storage
            inputFile.reset();
            outputFile.reset();
            remove("PRINTIPI_TEST_INPUT");
            remove("PRINTIPI_TEST_OUTPUT");
        }

        //@cmd g-code command to send to printer (a newline character will be appended)
        //@expect expected response
        void sendCommand(const std::string &cmd, const std::string &expect) {
            INFO("Sending command: '" + cmd + "'");
            *inputFile << cmd << '\n';
            inputFile->flush();
            INFO("It should be acknowledged with something that begins with '" + expect + "'");
            std::string got = readLine();
            REQUIRE(got.substr(0, expect.length()) == expect);
        }

        //Verify that the position as reported by the motion planner is near (@x, @y, @z)
        void verifyPosition(float x, float y, float z) const {
        	Vector4f actualPos = state.motionPlanner().actualCartesianPosition();
            INFO("Actual position: " + std::string(actualPos));
            REQUIRE(actualPos.xyz().distance(x, y, z) <= 4);
        }
        //Compare two std::chrono::durations, of potentially different types.
        template <typename AClock, typename ADur, typename BClock, typename BDur> 
            static void requireDurationsApproxEqual(const std::chrono::duration<AClock, ADur> &a, const std::chrono::duration<BClock, BDur> &b) {
            typedef std::chrono::duration<AClock, ADur> A;
            typedef std::chrono::duration<BClock, BDur> B;
            typedef typename std::common_type<A, B>::type Common;
            Common smaller = std::min(Common(a), Common(b));
            Common bigger = std::max(Common(a), Common(b));
            INFO("times should be approx equal: ");
            INFO(std::to_string(smaller.count()));
            INFO(std::to_string(bigger.count()));
            bool timesAreApproxEqual = bigger <= smaller + Common(std::chrono::microseconds(1)) && 
              smaller >= bigger + Common(std::chrono::microseconds(-1));
            REQUIRE(timesAreApproxEqual);
            //REQUIRE(bigger <= smaller + Common(std::chrono::microseconds(1)));
            //REQUIRE(smaller >= bigger + Common(std::chrono::microseconds(-1)));
            //REQUIRE(Common(a).count() == Approx(Common(b).count()));
        }
        //Compare two std::chrono::time_points, of potentially different types.
        template <typename AClock, typename ADur, typename BClock, typename BDur> 
            static void requireTimesApproxEqual(const std::chrono::time_point<AClock, ADur> &a, const std::chrono::time_point<BClock, BDur> &b) {
            requireDurationsApproxEqual(a.time_since_epoch(), b.time_since_epoch());
        }

        void exitOnce() {
            //check if the eventThread represents a valid thread
            if (eventThread.joinable()) {
                sendCommand("M0", "ok");
                eventThread.join();
            }
        }
        void threadedEventLoop() {
            eventThread = std::thread([&](){ 
                state.eventLoop(); 
            });
        }

        //convenience function to read and wait for the next line from Printipi's output
        std::string readLine() {
            std::string read;
            char curChar = 0;
            while (curChar != '\n') {
                if (outputFile->readsome(&curChar, 1) && curChar != '\n') {
                    read += curChar;
                } 
            }
            return read;
        };          
};

template <typename MachineT, typename ... Args> TestHelper<MachineT> makeTestHelper(const MachineT &machine, Args ...args) {
    return TestHelper<MachineT>(machine, args...);
}

/*
In gcc-4.6, the following results in "sorry, unimplemented: cannot expand ‘Args ...’ into a fixed-length argument list"
So use the workaround further below.
template <typename ... Args> TestMachine<Args...> makeTestMachine(Args ...args) {
    return TestMachine<Args...>(args...);
}*/
template <typename GetIoDrivers=DefaultGetIoDrivers, typename GetCoordMap=DefaultGetCoordMap, 
typename GetAccelerationProfile=DefaultGetAccelerationProfile> TestMachine<GetIoDrivers, GetCoordMap, GetAccelerationProfile> 
    makeTestMachine(const GetIoDrivers &getIoDrivers=GetIoDrivers(), const GetCoordMap &getCoordMap=GetCoordMap(), 
      const GetAccelerationProfile &getAccelerationProfile=GetAccelerationProfile()) {
    return TestMachine<GetIoDrivers, GetCoordMap, GetAccelerationProfile>(getIoDrivers, getCoordMap, getAccelerationProfile);
}


#endif