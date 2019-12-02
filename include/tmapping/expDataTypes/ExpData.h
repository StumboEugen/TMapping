//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPData_H
#define TMAPPING_EXPData_H

#include <vector>
#include <memory>

#include "../gateTypes/GateTypes.h"
#include "../landmarkTypes/LandmarkTypes.h"

namespace tmap
{

enum class ExpType{Intersection, Corridor, Stair, BigRoom, SmallRoom};

/// 代表观测得到的一次地形数据, 比如一个路口, 一个房间的信息
class ExpData
{
    std::vector<GateUnPtr> gates;
    std::vector<PLMUnPtr> posLandmarks;

public:
    virtual ExpType type() = 0;

    const std::vector<GateUnPtr>& getGates() const;

    void addGate(GateUnPtr pGate);

    void addLandmark(PLMUnPtr pLandmark);

};

}


#endif //TMAPPING_EXPData_H
