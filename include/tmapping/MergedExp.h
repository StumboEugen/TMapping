//
// Created by stumbo on 2019/12/2.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
#define TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H

#include <vector>

#include "tools/TopoParams.h"
#include "expDataTypes/ExpDataTypes.h"

namespace tmap
{

class MergedExp
{
    std::vector<ExpDataPtr> relatedExps;
    std::vector<MapTwigWePtr> relatedMaps;
    ExpDataPtr mergedExp;
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
