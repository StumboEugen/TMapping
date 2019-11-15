//
// Created by stumbo on 2019/11/15.
//

#include "StrPLM.h"

#include <utility>

tmap::StrPLM::StrPLM(const tmap::TopoVec2& pos, std::string str)
: PosLandmark(pos),
str(std::move(str))
{}

const std::string& tmap::StrPLM::getStr() const
{
    return str;
}
