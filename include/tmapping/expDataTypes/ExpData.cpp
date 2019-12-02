//
// Created by stumbo on 2019/11/15.
//

#include "ExpData.h"

using namespace tmap;

const std::vector<GateUnPtr>& tmap::ExpData::getGates() const
{
    return gates;
}

void ExpData::addGate(GateUnPtr pGate)
{
    gates.emplace_back(std::move(pGate));
}

void ExpData::addLandmark(PLMUnPtr pLandmark)
{
    posLandmarks.emplace_back(std::move(pLandmark));
}

