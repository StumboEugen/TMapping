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

