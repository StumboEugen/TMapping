//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_CORRIDOR_H
#define TMAPPING_CORRIDOR_H

#include "ExpData.h"

namespace tmap
{

class Corridor : public ExpData
{

    GateID mEndGateA = GATEID_CORRIDOR_NO_ENDPOINT;
    GateID mEndGateB = GATEID_CORRIDOR_NO_ENDPOINT;
    TopoVec2 mEndPointA{};
    TopoVec2 mEndPointB{};

public:

    ExpDataType type() const override { return ExpDataType::Corridor; }

    Json::Value toJS() const override;

    ExpDataPtr clone() override;

    GateID getEndGateA() const;

    GateID getEndGateB() const;

    const TopoVec2& getEndPointA() const;

    const TopoVec2& getEndPointB() const;

    void setEndGateA(GateID endGateA);

    void setEndGateB(GateID endGateB);

    void setEndPointA(const TopoVec2& endPointA);

    void setEndPointB(const TopoVec2& endPointB);

    std::array<double, 4> getOutBounding(double expandValue) const override;

    void moveGatePos(GateID id, const TopoVec2& newPos);

    TopoVec2 normalizeSelf() override;

    /**
     * @brief 一个帮助计算点击位置投影以及法向量的工具函数
     * @param C 点击的位置
     * @return first为C在AB上的投影位置, second为投影指向C的法向量
     */
    std::pair<TopoVec2, TopoVec2> calPosAmdNvFromPointC(const TopoVec2& C);

    /**
     * @brief 获取当前走廊的宽度的一半
     */
    double halfWidth() const;
};
}


#endif //TMAPPING_CORRIDOR_H
