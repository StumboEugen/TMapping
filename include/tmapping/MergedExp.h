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

/// TODO 检查是否可以构造出一种相互依赖的树形关系
class MergedExp
{
    /// 相关的几个Exp
    std::vector<ExpPtr> mRelatedExps;
    /// 使用这个MergedExp的Twig们(可能有的已经被废弃了)
    std::vector<MapTwigWePtr> mRelatedMaps;

    /// 构造的时候不会把LastSearchResult用RelatedMaps填满, 因此需要这个bool来判断last result空的时候代表什么
    bool hasSearchedLoopClosure = false;
    /// 每次的搜索不需要从头开始, 从末端开始就可以了
    std::vector<MapTwigWePtr> mLastSearchResult;

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
    double alike(const MergedExp& another) const;

    double alike(const ExpData& expData) const;

    size_t lastExpSerial() const;

    bool isLastButOneExpSerial(size_t serial2Check) const;

    /**
     * @brief 找到MOVE2NEW的末端Twig
     * @return 与此MergedExp可能的闭环的末端MapTwig
     */
    std::vector<MapTwigPtr> getLoopClosureMaps();

    MergedExp(const MergedExpPtr& father, const ExpPtr& newExp);
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
