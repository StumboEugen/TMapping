//
// Created by stumbo on 2019/11/15.
//

#include "Door.h"

tmap::Door::Door(const tmap::TopoVec2& pos, const tmap::TopoVec2& normalVec, bool oepned)
        : Gate(pos, normalVec), opened(oepned)
{}

bool tmap::Door::isOpened() const
{
    return opened;
}

Json::Value tmap::Door::toJS() const
{
    Json::Value res(Gate::toJS());
    res["opened"] = opened;
    res["mark"] = doorMark.getStr();
    res["type"] = "D";
    return res;
}
