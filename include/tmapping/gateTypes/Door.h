//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_DOOR_H
#define TMAPPING_DOOR_H

#include "Gate.h"
#include "../landmarkTypes/LandmarkTypes.h"

namespace tmap
{

class Door : public Gate
{
    bool opened;
    std::string doorMark;

public:
    Door(const TopoVec2& pos, const TopoVec2& normalVec, bool oepned, std::string doorMark = "");

    bool isOpened() const;

    GateType type() const override
    {
        return GateType::Door;
    }

    Json::Value toJS() const override;

    GatePtr clone() override;

    GatePtr newMergedGate(const GatePtr& that, const TopoVec2& thatPos, double thisWeight) const override;

    bool alike(const GatePtr& that) const override;
};
}


#endif //TMAPPING_DOOR_H
