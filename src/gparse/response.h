#ifndef GPARSE_RESPONSE
#define GPARSE_RESPONSE

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
