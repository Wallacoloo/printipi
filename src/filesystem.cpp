#include "filesystem.h"

FileSystem::FileSystem(const std::string &nBase) 
 : gcodeBase(nBase.empty() || nBase[nBase.size()-1] != '/' ? nBase + '/' : nBase) {}

std::string FileSystem::relGcodePathToAbs(const std::string &rel) {
    if (!rel.empty() && rel[0] == '/') {
        return gcodeBase + rel.substr(1); //drop the first / from rel.
    } else { //rel does not begin with a /
        return gcodeBase + rel;
    }
}
