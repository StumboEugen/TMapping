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
    return mGates;
}

void ExpData::addGate(GateUnPtr pGate)
{
    mGates.emplace_back(std::move(pGate));
}

void ExpData::addLandmark(PLMUnPtr pLandmark)
{
    posLandmarks.emplace_back(std::move(pLandmark));
}

GateID ExpData::findTheCloestGate(const TopoVec2& gatePos)
{
    size_t gateSize = mGates.size();
    if (gateSize == 0) {
        cerr << FILE_AND_LINE << " Gate size is ZERO ?!" << endl;
        throw;
    }
    auto lenMax = DBL_MAX;
    GateID res = 0;
    for (size_t i = 0; i < gateSize; ++i) {
        double currentLen = (mGates[i]->getPos() - gatePos).len();
        if (currentLen < lenMax) {
            lenMax = currentLen;
            res = i;
        }
    }

    return res;
}

MatchResult ExpData::detailedMatch(const ExpData& another, double selfWeight) const
{
    /// TODO 完成两个未对齐数据的匹配工作
    /// 要求: MatchResult->gatesMapping是从 [another's gate] = this's gate
    /// MatchResult->mergedExpData 的gate序号和another完全相同
    /// selfWeight指的是this的权重, 比如selfWeight=3, 说明this可能是3次结果融合而成的

}

double ExpData::quickMatch(const ExpData& another, double selfWeight) const
{
    /// 快速的数据匹配工作
    if (another.type() == this->type()) {
        return 1.0;
    } else {
        return 0.0;
    }
}

size_t ExpData::nGates() const
{
    return mGates.size();
}

