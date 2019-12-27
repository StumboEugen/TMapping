//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_POSLANDMARK_H
#define TMAPPING_POSLANDMARK_H

#include "Landmark.h"
#include "../tools/TopoVec2.h"

namespace tmap
{

class PosLandmark;

using PLMPtr = std::shared_ptr<PosLandmark>;
using PLMWePtr = std::weak_ptr<PosLandmark>;

class PosLandmark : public Landmark
{
    TopoVec2 pos;

public:
    static PLMPtr madeFromJS(const Jsobj& jPLM);

    explicit PosLandmark(const TopoVec2& pos);

    LandmarkType type() const override = 0 ;

    const TopoVec2& getPos() const;

    Json::Value toJS() const override;

    virtual PLMPtr clone() = 0;
};
}


#endif //TMAPPING_POSLANDMARK_H
