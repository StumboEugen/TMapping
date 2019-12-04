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
    std::vector<ExpPtr> relatedExps;
    std::vector<MapTwigWePtr> relatedMaps;

    class Fast2DGateID_ {
        size_t mDimX;
        size_t mDimY;
        std::vector<gateID> data;

    public:

        gateID& at(size_t x, size_t y);

        const gateID& at(size_t x, size_t y) const;

        Fast2DGateID_(size_t mDimX, size_t mDimY);
    };
    /// 令k = mGatesMapping.at(2,3) 意味着
    /// mergedExpData->gates[3] 和 relatedExp->gates[2] 对应同一个 Gate
    Fast2DGateID_ mGatesMapping;

    /// 融合后的实际ExpData数据
    /// @note 如果是单个ExpData融合而成的, 这个ptr指向的就是原始的ExpDataPtr
    ExpDataPtr mergedExpData;

public:
    double alike(const MergedExp& another);

    double alike(const ExpData& expData);
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
