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

    /// 如果为0, 表示完全不限制地图生长
    size_t nSurviverMaps = 200;

public:
    void setLeftGate(GateID gateID);

    void arriveNewExp(const ExpPtr& newExp);

    Jsobj getTopMaps(size_t nTops);

    void setNSurviverMaps(size_t nMaps);
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_TOPOMAPPING_H
