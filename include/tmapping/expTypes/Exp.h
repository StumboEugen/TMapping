//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXP_H
#define TMAPPING_EXP_H

#include <vector>
#include <memory>
#include "tmapping/gateTypes/GateTypes.h"

namespace tmap
{

enum class ExpType{Intersection, Corridor, Stair, BigRoom, SmallRoom};

class Exp
{
    // 离开时对应的Gate, -1表示还没离开
    int leftGate = -1;
    std::vector<GatePtr> gates;

public:
    virtual ExpType type() = 0;

    void setLeftGate(int leftGate);

    int getLeftGate() const;

    const std::vector<GatePtr>& getGates() const;

    void addGate(GatePtr pGate);

};

}


#endif //TMAPPING_EXP_H
