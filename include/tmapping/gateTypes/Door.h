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
    bool oepned;
    StrLM doorMark;

public:
    Door(const TopoVec2& pos, const TopoVec2& normalVec, bool oepned);

    bool isOepned() const;

    GateType type() override
    {
        return GateType::Door;
    }
};
}


#endif //TMAPPING_DOOR_H
