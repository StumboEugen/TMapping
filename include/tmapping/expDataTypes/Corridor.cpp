//
// Created by stumbo on 2019/11/15.
//

#include "Corridor.h"


Json::Value tmap::Corridor::toJS() const
{
    Json::Value res(ExpData::toJS());
    res["endG_A"] = mEndGateA;
    res["endG_B"] = mEndGateB;
    res["endP_A"] = std::move(mEndPointA.toJS());
    res["endP_B"] = std::move(mEndPointB.toJS());
    return res;
}

tmap::ExpDataPtr tmap::Corridor::clone()
{
    auto c = new Corridor;
    this->copy2(c);
    c->mEndGateA = this->mEndGateA;
    c->mEndGateB = this->mEndGateB;
    return tmap::ExpDataPtr(c);
}

tmap::GateID tmap::Corridor::getEndGateA() const
{
    return mEndGateA;
}

tmap::GateID tmap::Corridor::getEndGateB() const
{
    return mEndGateB;
}

void tmap::Corridor::setEndGateA(tmap::GateID endPointA)
{
    mEndGateA = endPointA;
    if (endPointA > 0) {
        mEndPointA = getGates()[endPointA]->getPos();
    }
}

void tmap::Corridor::setEndGateB(tmap::GateID endPointB)
{
    mEndGateB = endPointB;
    if (endPointB > 0) {
        mEndPointA = getGates()[endPointB]->getPos();
    }
}

const tmap::TopoVec2& tmap::Corridor::getEndPointA() const
{
    return mEndPointA;
}

const tmap::TopoVec2& tmap::Corridor::getEndPointB() const
{
    return mEndPointB;
}

void tmap::Corridor::setEndPointA(const tmap::TopoVec2& endPointA)
{
    mEndPointA = endPointA;
}

void tmap::Corridor::setEndPointB(const tmap::TopoVec2& endPointB)
{
    mEndPointB = endPointB;
}
