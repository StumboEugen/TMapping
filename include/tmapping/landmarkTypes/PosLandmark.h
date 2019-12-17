//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_POSLANDMARK_H
#define TMAPPING_POSLANDMARK_H

#include "Landmark.h"
#include "../tools/TopoVec2.h"

namespace tmap
{

class PosLandmark : public Landmark
{
    TopoVec2 pos;

public:

    explicit PosLandmark(const TopoVec2& pos);

    LandmarkType type() override = 0 ;

    const TopoVec2& getPos() const;

    Json::Value toJS() const override;
};
}


#endif //TMAPPING_POSLANDMARK_H
