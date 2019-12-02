//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPDATATYPES_H
#define TMAPPING_EXPDATATYPES_H

#include "ExpData.h"
#include "Corridor.h"
#include "Intersection.h"
#include "SmallRoom.h"

namespace tmap
{
    using ExpDataPtr = std::shared_ptr<ExpData>;
    using ExpDataUnPtr = std::unique_ptr<ExpData>;
    using ExpDataWePtr = std::weak_ptr<ExpData>;
}


#endif //TMAPPING_EXPDATATYPES_H
