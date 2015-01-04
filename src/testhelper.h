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

template <typename MachineT> class TestHelper {
    //must be unique_ptr in order to be movable
    std::unique_ptr<std::ofstream> inputFile;
    std::unique_ptr<std::ifstream> outputFile;
    MachineT driver;
    FileSystem fs;
    gparse::Com com;
    State<MachineT> state;
    std::thread eventThread;
    public:
        TestHelper(const MachineT &machine=MachineT(), bool enterEventLoop=true) 
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
          driver(machine), fs("./"), com("PRINTIPI_TEST_INPUT", "PRINTIPI_TEST_OUTPUT"), state(driver, fs, com, true) {
            //Note: the above inputFile and outputFile MUST be opened before com is instantiated, otherwise the files may not have been created
            //  and com will have a null file handle.   
            if (enterEventLoop) {
                this->threadedEventLoop();
            }      
        }
        TestHelper(TestHelper<MachineT> &&) = default;

        ~TestHelper() {
            exitOnce();
            remove("PRINTIPI_TEST_INPUT");
            remove("PRINTIPI_TEST_OUTPUT");
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

        //@cmd g-code command to send to printer (a newline character will be appended)
        //@expect expected response
        void sendCommand(const std::string &cmd, const std::string &expect) {
            INFO("Sending command: '" + cmd + "'");
            *inputFile << cmd << '\n';
            INFO("It should be acknowledged with something that begins with '" + expect + "'");
            std::string got = readLine();
            REQUIRE(got.substr(0, expect.length()) == expect);
        };

        //Verify that the position as reported by the motion planner is near (@x, @y, @z)
        void verifyPosition (float x, float y, float z) const {
        	Vector4f actualPos = state.motionPlanner().actualCartesianPosition();
            INFO("Actual position: " + std::string(actualPos));
            REQUIRE(actualPos.xyz().distance(x, y, z) <= 4);
        };

        void exitOnce() {
            //check if the eventThread represents a valid thread
            if (eventThread.joinable()) {
                sendCommand("M0", "ok");
                eventThread.join();
            }
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