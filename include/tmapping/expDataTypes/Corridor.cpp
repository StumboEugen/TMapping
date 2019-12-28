//
// Created by stumbo on 2019/11/15.
//

#include "Corridor.h"


Json::Value tmap::Corridor::toJS() const
{
    Json::Value res(ExpData::toJS());
    res["endP_A"] = mEndPointA;
    res["endP_B"] = mEndPointB;
    return res;
}

tmap::ExpDataPtr tmap::Corridor::clone()
{
    auto c = new Corridor;
    this->copy2(c);
    c->mEndPointA = this->mEndPointA;
    c->mEndPointB = this->mEndPointB;
    return tmap::ExpDataPtr(c);
}

tmap::GateID tmap::Corridor::getEndPointA() const
{
    return mEndPointA;
}

tmap::GateID tmap::Corridor::getEndPointB() const
{
    return mEndPointB;
}

void tmap::Corridor::setEndPointA(tmap::GateID endPointA)
{
    mEndPointA = endPointA;
}

void tmap::Corridor::setEndPointB(tmap::GateID endPointB)
{
    mEndPointB = endPointB;
}
