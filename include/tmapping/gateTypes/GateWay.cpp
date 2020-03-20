//
// Created by stumbo on 2019/11/15.
//

#include "GateWay.h"

Json::Value tmap::GateWay::toJS() const
{
    Json::Value res(Gate::toJS());
    return res;
}

tmap::GatePtr tmap::GateWay::clone()
{
    auto gateWay = new GateWay(this->getPos(), this->getNormalVec());
    gateWay->setPossibility(this->getPossibility());
    return GatePtr{gateWay};
}

bool tmap::GateWay::alike(const tmap::GatePtr& that) const
{
    return Gate::alike(that);
}

tmap::GatePtr
tmap::GateWay::newMergedGate(const tmap::GatePtr& that, const tmap::TopoVec2& thatPos,
                             double thisWeight) const
{
    auto gateWay = new GateWay({0,0}, {1,0});
    gateWay->mergeBasicInfo(this, that.get(), thatPos, thisWeight);
    return GatePtr{gateWay};
}
