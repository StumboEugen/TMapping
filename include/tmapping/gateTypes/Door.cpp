//
// Created by stumbo on 2019/11/15.
//

#include <iostream>

#include "Door.h"

using namespace std;
using namespace tmap;

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

bool tmap::Door::alike(const tmap::GatePtr& that) const
{
    switch (that->type()) {
        case GateType::GateWay:
            return false;
        case GateType::DoorOpened:
        case GateType::DoorClosed:
            cout << FILE_AND_LINE << " a shouldn't shown GateType: " << (int)that->type() <<
            endl;
        case GateType::Door:
            break;
        default:
            cout << FILE_AND_LINE << " an unknown GateType: " << (int)that->type() << endl;
            return false;
    }

    auto realThat = (Door*)(that.get());
    return Gate::alike(that) && this->doorMark == realThat->doorMark;
}

GatePtr
tmap::Door::newMergedGate(const GatePtr& that, const TopoVec2& thatPos, double thisWeight) const
{
    auto res = new Door(
            {0,0},
            {1,0},
            this->opened,
            this->doorMark);
    res->mergeBasicInfo(this, that.get(), thatPos, thisWeight);
    return GatePtr{res};
}
