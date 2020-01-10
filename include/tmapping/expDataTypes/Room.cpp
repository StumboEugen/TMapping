//
// Created by stumbo on 2019/11/15.
//

#include "Room.h"

Json::Value tmap::Room::toJS() const
{
    Json::Value res(ExpData::toJS());
    return res;
}

tmap::ExpDataPtr tmap::Room::clone()
{
    auto sr = new Room;
    this->copy2(sr);
    return tmap::ExpDataPtr(sr);
}
