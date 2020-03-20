//
// Created by stumbo on 2019/11/15.
//

#include "Room.h"

Json::Value tmap::Room::toJS() const
{
    Json::Value res(ExpData::toJS());
    res["scaling"] = mScaling;
    return res;
}

tmap::ExpDataPtr tmap::Room::clone()
{
    auto sr = new Room;
    this->copy2(sr);
    sr->mScaling = this->mScaling;
    return tmap::ExpDataPtr(sr);
}

void tmap::Room::setScaling(double scaling)
{
    Room::mScaling = scaling;
}

double tmap::Room::getScaling() const
{
    return mScaling;
}

tmap::ExpDataPtr tmap::Room::cloneShell() const
{
    auto sr = new Room;
    sr->mScaling = this->mScaling;
    return tmap::ExpDataPtr(sr);
}
