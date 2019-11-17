//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_GATETYPES_H
#define TMAPPING_GATETYPES_H

#include "Gate.h"
#include "GateWay.h"
#include "Door.h"

#include <memory>

namespace tmap
{
    using GatePtr = std::shared_ptr<Gate>;
    using GateUnPtr = std::unique_ptr<Gate>;
    using GateWePtr = std::weak_ptr<Gate>;
}

#endif //TMAPPING_GATETYPES_H
