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

    /// 融合后的实际Exp数据
    /// @warning 如果是单个Exp融合而成的, 这个ptr指向的就是原始的ExpDataPtr
    ExpDataPtr mergedExpData;

public:
    double alike(const MergedExp& another);

    double alike(const ExpData& expData);
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
