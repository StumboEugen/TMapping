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
    StructedMap mChampionMap;

    size_t mSurviverSetting = 0;

public:
    void setLeftGate(GateID gateID);

    void arriveNewExp(const ExpPtr& newExp);

    Jsobj getTopMaps(size_t nTops);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_TOPOMAPPING_H
