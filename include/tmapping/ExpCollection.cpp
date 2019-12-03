//
// Created by stumbo on 2019/11/15.
//

#include "ExpCollection.h"
#include "Exp.h"

void tmap::ExpCollection::setLeftGateOfCurrent(size_t leftGate)
{
    experiences.back()->leftGate = leftGate;
}

void tmap::ExpCollection::setLeftGateOfCurrent(const TopoVec2& gatePos)
{
    auto cloestGate = experiences.back()->data->findTheCloestGate(gatePos);
    setLeftGateOfCurrent(cloestGate);
}
