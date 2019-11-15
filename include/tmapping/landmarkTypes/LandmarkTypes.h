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
    using LMPtr = std::unique_ptr<Landmark>;
    using PLMPtr = std::unique_ptr<PosLandmark>;
}

#endif //TMAPPING_LANDMARKTYPES_H
