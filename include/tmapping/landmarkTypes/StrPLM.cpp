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

bool tmap::StrPLM::alike(const tmap::LMPtr& that) const
{
    return that->type() == LandmarkType::StrPLM && this->str == ((StrPLM*)that.get())->str;
}

tmap::PLMPtr tmap::StrPLM::newMergedPLM(const tmap::PLMPtr& that,
        const tmap::TopoVec2& thatPos, double thisWeight) const
{
    auto newPos = (this->getPos() * thisWeight + thatPos) / (thisWeight + 1);
    auto newsp = new StrPLM(newPos, this->str);
    return PLMPtr{newsp};
}
