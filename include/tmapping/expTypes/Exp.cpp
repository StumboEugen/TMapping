//
// Created by stumbo on 2019/11/15.
//

#include "Exp.h"

using namespace tmap;

void tmap::Exp::setLeftGate(int leftGate)
{
    Exp::leftGate = leftGate;
}

int tmap::Exp::getLeftGate() const
{
    return leftGate;
}

const std::vector<GateUnPtr>& tmap::Exp::getGates() const
{
    return gates;
}

void Exp::addGate(GateUnPtr pGate)
{
    gates.emplace_back(std::move(pGate));
}

void Exp::addLandmark(PLMUnPtr pLandmark)
{
    posLandmarks.emplace_back(std::move(pLandmark));
}

