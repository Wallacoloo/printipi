#ifndef COMMON_FILESYSTEM_H
#define COMMON_FILESYSTEM_H

#include <string>

class FileSystem {
    std::string gcodeBase;
    public:
        FileSystem(const std::string &nBase);
        std::string relGcodePathToAbs(const std::string &rel);
};


#endif
