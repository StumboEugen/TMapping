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

Json::Value tmap::StrPLM::toJS() const
{
    Json::Value res(PosLandmark::toJS());
    res["str"] = str;
    return res;
}

tmap::PLMPtr tmap::StrPLM::clone()
{
    auto newsp = new StrPLM(this->getPos(), this->str);
    return PLMPtr{newsp};
}
