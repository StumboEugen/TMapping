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
    // 进入时对应的Gate, -1表示这个为出发点
    int32_t enterGate = -1;
    // 离开时对应的Gate, -1表示还没离开
    int32_t leftGate = -1;
    std::vector<GateUnPtr> gates;
    std::vector<PLMUnPtr> posLandmarks;

public:
    virtual ExpType type() = 0;

    void setLeftGate(int32_t leftGate);

    int32_t getLeftGate() const;

    void setEnterGate(int32_t enterGate);

    int32_t getEnterGate() const;

    const std::vector<GateUnPtr>& getGates() const;

    void addGate(GateUnPtr pGate);

    void addLandmark(PLMUnPtr pLandmark);

};

}


#endif //TMAPPING_EXP_H
