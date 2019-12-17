//
// Created by stumbo on 2019/11/15.
//

#include "Corridor.h"

//tmap::Corridor::Corridor(bool completed) : completed(completed)
//{}

Json::Value tmap::Corridor::toJS() const
{
    Json::Value res(ExpData::toJS());
    res["type"] = "C";
    return res;
}
