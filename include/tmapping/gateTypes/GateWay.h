//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_GATEWAY_H
#define TMAPPING_GATEWAY_H

#include "Gate.h"

namespace tmap
{

class GateWay : public Gate
{

public:
    GateWay(const TopoVec2& pos, const TopoVec2& normalVec) : Gate(pos, normalVec)
    {}

    GateType type() const override
    {
        return GateType::GateWay;
    }

    Json::Value toJS() const override;

    GatePtr clone() override;
};

}


#endif //TMAPPING_GATEWAY_H
