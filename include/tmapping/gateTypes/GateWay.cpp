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
