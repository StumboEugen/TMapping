//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPDATA_H
#define TMAPPING_EXPDATA_H

#include <vector>
#include <memory>

#include "../gateTypes/GateTypes.h"
#include "../landmarkTypes/LandmarkTypes.h"

namespace tmap
{

enum class ExpDataType{Intersection, Corridor, Stair, BigRoom, SmallRoom};

/// 代表观测得到的一次地形数据, 比如一个路口, 一个房间的信息
class ExpData
{
    std::vector<GateUnPtr> gates;
    std::vector<PLMUnPtr> posLandmarks;

public:
    virtual ExpDataType type() const = 0;

    const std::vector<GateUnPtr>& getGates() const;

    void addGate(GateUnPtr pGate);

    void addLandmark(PLMUnPtr pLandmark);

    size_t findTheCloestGate(const TopoVec2& gatePos);

    /**
     * @brief 计算两个ExpData是否类似
     * @param another 另一个用于比较的ExpData实例
     * @param selfWeight this的比重, 1:1就是1, 1:4就是0.25, 5:1就是5
     * @return 可能性系数, 如果为0表示完全不可能
     */
    double alike(const ExpData& another, double selfWeight = 1.0) const;
};

}


#endif //TMAPPING_EXPDATA_H
