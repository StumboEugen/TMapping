//
// Created by stumbo on 2019/12/3.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_TOPOMAPPING_H
#define TMAPPING_INCLUDE_TMAPPING_TOPOMAPPING_H

#include "ExpCollection.h"
#include "MapTwigCollection.h"
#include "tools/TopoParams.h"
#include "tools/TopoVec2.h"

namespace tmap
{

class TopoMapping
{
    ExpCollection mExperiences;
    MapTwigCollection twigCollection;

public:
    void addNewExp(ExpDataPtr newExpData);

    void setLeftGate(size_t gateID);
    void setLeftGate(TopoVec2 gatePos);

    void arriveNewExp(ExpPtr newExp);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_TOPOMAPPING_H