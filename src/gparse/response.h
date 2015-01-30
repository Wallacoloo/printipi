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


#ifndef GPARSE_RESPONSE_H
#define GPARSE_RESPONSE_H

#include <string>
#include <sstream>
#include <utility> //for std::pair
#include <initializer_list>

namespace gparse {

enum ResponseCode {
    ResponseOk,
    ResponseNull,
    ResponseWarning,
};

/* 
 * Response objects encapsulate a single response from the client (us) to a gcode Command sent by the host.
 * When converted to strings, they typically appear like "ok" or "ok T:153.7"
 *
 * It was decided to make a thin wrapper around the raw string responses in order to standardize the 'ok', '!!' (error), etc response prefixes.
 */
class Response {
    ResponseCode code;
    std::string rest;
    public:
        // Response::Ok provides easy access to a response formatted as "ok"
        static const Response Ok;
        // Response::NULL allows one to indicate that no response should be sent
        static const Response Null;

        //Construct a response from a code, followed by an optional extra string (implicitly joined by a space)
        inline Response(ResponseCode code, const std::string &rest="") : code(code), rest(rest) {
        }
        //Construct a response from a code, followed by an optional extra C string (implicitly joined by a space)
        inline Response(ResponseCode code, const char* rest) : code(code), rest(rest) {
        }

        //Construct a response from a code, a set of Key:Value pairs, and then an extra string (all 3 are joined by spaced)
        //@pairs is given as any container whose elements are std::pairs<std::string, std::string>,
        //  in which std::pair::first is the key, and std::pair::second is the value.
        template <typename Container> Response(ResponseCode code, const Container &pairs, const std::string &rest="")
          : code(code), rest(joinPairsAndStr(pairs, rest)) {}

        //Construct a response from a code and a set of Key:Value pairs (joined by a space)
        //Allow construction, using an std::initializer_list of std::pair<std::string, std::string> (or const char*, etc) for @pairs.
        //Example: Response(ResponseOk, {make_pair("T", "65"), make_pair("B", "20")})
        //This specialization is only needed in gcc-4.6, where automatic deduction of a std::initializer_list as Container would cause a warning in the other version
        template <typename T> Response(ResponseCode code, std::initializer_list<T> pairs, const std::string &rest="")
          : code(code), rest(joinPairsAndStr(pairs, rest)) {}

        //Convert the Response object to a string
        //Note: no newline character is appended to the end; the string is a single line of text.
        inline std::string toString() const {
            switch (code) {
                case ResponseOk:
                    return "ok" + (rest.empty() ? "" : " " + rest);
                case ResponseWarning:
                    return "// warning: " + rest;
                case ResponseNull:
                default:
                    return rest;
            }
        }
        inline bool isNull() const {
            return code == ResponseNull;
        }
    private:
        template <typename Container> std::string joinPairsAndStr(const Container &pairs, const std::string &append) {
            std::ostringstream imploded;
            bool first=true;
            for (const auto& elem : pairs) {
                //to join each pair by a space, prepend each element EXCEPT the first with a space.
                if (first) {
                    first = false;
                } else {
                    imploded << ' ';
                }
                imploded << elem.first << ':' << elem.second;
            }
            if (!append.empty()) {
                if (!first) {
                    //join the two strings with a space, but only if neither of them are empty
                    imploded << ' ';
                }
                imploded << append;
            }
            return imploded.str();
        }
};


}
#endif
