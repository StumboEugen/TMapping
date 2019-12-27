//
// Created by stumbo on 2019/11/15.
//

#include "Corridor.h"

//tmap::Corridor::Corridor(bool completed) : completed(completed)
//{}

Json::Value tmap::Corridor::toJS() const
{
    Json::Value res(ExpData::toJS());
    return res;
}

tmap::ExpDataPtr tmap::Corridor::clone()
{
    auto c = new Corridor;
    this->copy2(c);
    c->completed = this->completed;
    return tmap::ExpDataPtr(c);
}
