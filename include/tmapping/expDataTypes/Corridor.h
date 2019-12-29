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
};
}


#endif //TMAPPING_CORRIDOR_H
