//
// Created by stumbo on 2019/11/15.
//

#include "StrLM.h"

#include <utility>

const std::string& tmap::StrLM::getStr() const
{
    return str;
}

tmap::StrLM::StrLM(std::string str) : str(std::move(str))
{}

Json::Value tmap::StrLM::toJS() const
{
    Json::Value res(Landmark::toJS());
    res["str"] = str;
    return res;
}

bool tmap::StrLM::alike(const tmap::LMPtr& that) const
{
    return that->type() == LandmarkType::StrLM && this->str == ((StrLM*)that.get())->str;
}

