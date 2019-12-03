//
// Created by stumbo on 2019/11/15.
//

#include "ExpCollection.h"
#include "Exp.h"

void tmap::ExpCollection::setLeftGateOfCurrent(size_t leftGate)
{
    mExperiencesData.back()->mLeftGate = leftGate;
}

void tmap::ExpCollection::setLeftGateOfCurrent(const TopoVec2& gatePos)
{
    auto cloestGate = mExperiencesData.back()->mData->findTheCloestGate(gatePos);
    setLeftGateOfCurrent(cloestGate);
}

size_t tmap::ExpCollection::registExp(tmap::ExpPtr expPtr)
{
    expPtr->nSerial = mExperiencesData.size();
    mExperiencesData.push_back(std::move(expPtr));
    return mExperiencesData.size();
}
