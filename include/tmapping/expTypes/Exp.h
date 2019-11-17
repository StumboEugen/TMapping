//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXP_H
#define TMAPPING_EXP_H

#include <vector>
#include <memory>

#include "../gateTypes/GateTypes.h"
#include "../landmarkTypes/LandmarkTypes.h"

namespace tmap
{

enum class ExpType{Intersection, Corridor, Stair, BigRoom, SmallRoom};

class Exp
{
    // 离开时对应的Gate, -1表示还没离开
    int leftGate = -1;
    std::vector<GateUnPtr> gates;
    std::vector<PLMUnPtr> posLandmarks;

public:
    virtual ExpType type() = 0;

    void setLeftGate(int leftGate);

    int getLeftGate() const;

    const std::vector<GateUnPtr>& getGates() const;

    void addGate(GateUnPtr pGate);

    void addLandmark(PLMUnPtr pLandmark);

};

}


#endif //TMAPPING_EXP_H
