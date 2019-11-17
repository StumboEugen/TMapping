//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_LANDMARKTYPES_H
#define TMAPPING_LANDMARKTYPES_H

#include <memory>

#include "Landmark.h"
#include "PosLandmark.h"
#include "StrLM.h"
#include "NumLM.h"
#include "StrPLM.h"

namespace tmap
{
    using LMPtr = std::shared_ptr<Landmark>;
    using LMUnPtr = std::unique_ptr<Landmark>;
    using LMWePtr = std::weak_ptr<Landmark>;

    using PLMPtr = std::shared_ptr<PosLandmark>;
    using PLMUnPtr = std::unique_ptr<PosLandmark>;
    using PLMWePtr = std::weak_ptr<PosLandmark>;
}

#endif //TMAPPING_LANDMARKTYPES_H
