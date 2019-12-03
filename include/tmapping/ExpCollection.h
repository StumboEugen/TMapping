//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPCOLLECTION_H
#define TMAPPING_EXPCOLLECTION_H

#include "expDataTypes/ExpDataTypes.h"
#include "tools/TopoTools.h"
#include <memory>
#include <unordered_map>
#include <set>

namespace tmap
{

class ExpCollection
{
    std::vector<ExpPtr> experiences;
    std::unordered_map<int, std::set<Intersection*>> intersections;
    // TODO 合适的分类标准

public:
    void setLeftGateOfCurrent(size_t leftGate);
    void setLeftGateOfCurrent(const TopoVec2& gatePos);
};
}


#endif //TMAPPING_EXPCOLLECTION_H
