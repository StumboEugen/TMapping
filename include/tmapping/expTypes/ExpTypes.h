//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPTYPES_H
#define TMAPPING_EXPTYPES_H

#include "Exp.h"
#include "Corridor.h"
#include "Intersection.h"
#include "SmallRoom.h"

namespace tmap
{
    using ExpPtr = std::unique_ptr<Exp>;
}


#endif //TMAPPING_EXPTYPES_H
