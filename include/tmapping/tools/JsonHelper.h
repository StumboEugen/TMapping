//
// Created by stumbo on 2019/12/18.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_TOOLS_JSONHELPER_H
#define TMAPPING_INCLUDE_TMAPPING_TOOLS_JSONHELPER_H

#include "TopoParams.h"
#include <string>

namespace tmap
{

namespace JsonHelper
{

std::string JS2Str(const Jsobj& js, bool shortVersion = true, uint8_t precision = 4);

Jsobj Str2JS(const std::string& str);

int saveJson(const Jsobj& js, std::string fileName = "", bool addTime = false);

Jsobj loadJson(const std::string& fileName);

};
}


#endif //TMAPPING_INCLUDE_TMAPPING_TOOLS_JSONHELPER_H
