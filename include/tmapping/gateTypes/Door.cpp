//
// Created by stumbo on 2019/11/15.
//

#include "Door.h"

tmap::Door::Door(const tmap::TopoVec2& pos, const tmap::TopoVec2& normalVec, bool oepned,
                 std::string doorMark)
        : Gate(pos, normalVec),
          opened(oepned),
          doorMark(std::move(doorMark))
{}

bool tmap::Door::isOpened() const
{
    return opened;
}

Json::Value tmap::Door::toJS() const
{
    Json::Value res(Gate::toJS());
    res["opened"] = opened;
    res["mark"] = doorMark;
    return res;
}

tmap::GatePtr tmap::Door::clone()
{
    auto cloned = new Door(
            this->getPos(),
            this->getNormalVec(),
            this->opened,
            this->doorMark);
    cloned->setPossibility(this->getPossibility());
    return GatePtr{cloned};
}
