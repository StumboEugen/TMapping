//
// Created by stumbo on 2019/11/15.
//

#include <cfloat>
#include "ExpData.h"
#include <iostream>
#include "../tools/TopoParams.h"

using namespace std;
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

size_t ExpData::findTheCloestGate(const TopoVec2& gatePos)
{
    size_t gateSize = gates.size();
    if (gateSize == 0) {
        cerr << FILE_AND_LINE << " Gate size is ZERO ?!" << endl;
        throw;
    }
    auto lenMax = DBL_MAX;
    size_t res = 0;
    for (size_t i = 0; i < gateSize; ++i) {
        double currentLen = (gates[i]->getPos() - gatePos).len();
        if (currentLen < lenMax) {
            lenMax = currentLen;
            res = i;
        }
    }

    return res;
}

double ExpData::alike(const ExpData& another, double selfWeight) const
{
    /// TODO 完成两个未对齐数据的匹配工作
    if (another.type() == this->type()) {
        return 1.0;
    } else {
        return 0.0;
    }
}

