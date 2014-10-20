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
 * gparse/response.h
 *
 * Response objects encapsulate a single response from the client (us) to a gcode Command sent by the host.
 * When converted to strings, they typically appear like "ok" or "ok T:153.7"
 *
 * It was decided to make a thin wrapper around the raw string responses in order to standardize the 'ok', '!!' (error), etc response prefixes.
 *
 * Response::Ok provides easy access to a response formatted as "ok"
 */

#ifndef GPARSE_RESPONSE_H
#define GPARSE_RESPONSE_H

#include <string>

namespace gparse {

enum ResponseCode {
    ResponseOk,
    ResponseNull
};

class Response {
    ResponseCode code;
    std::string rest;
    public:
        static const Response Ok;
        static const Response Null;
        inline Response(ResponseCode nCode) : code(nCode) {
        }
        inline Response(ResponseCode nCode, const std::string &nRest) : code(nCode), rest(nRest) {
        }
        inline std::string toString() const {
            return (code == ResponseOk ? "ok" : "") + (rest.empty() ? "" : " " + rest) + "\n";
        }
        inline bool isNull() {
            return code == ResponseNull;
        }
};


}
#endif
