//
// Created by stumbo on 2019/11/15.
//

#include "SmallRoom.h"

Json::Value tmap::SmallRoom::toJS() const
{
    Json::Value res(ExpData::toJS());
    return res;
}

tmap::ExpDataPtr tmap::SmallRoom::clone()
{
    auto sr = new SmallRoom;
    this->copy2(sr);
    return tmap::ExpDataPtr(sr);
}
