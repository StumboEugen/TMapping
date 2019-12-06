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
    MergedExpPtr mFather;
    /// 在此mergedExp新加入的Exp
    ExpPtr mRelatedExp;
    /// 使用这个MergedExp的Twig们(可能有的已经被废弃了)
    std::vector<MapTwigWePtr> mRelatedMaps;

    /// 构造的时候不会把LastSearchResult用RelatedMaps填满, 因此需要这个bool来判断last result空的时候代表什么
    bool hasSearchedLoopClosure = false;
    /// 每次的搜索不需要从头开始, 从末端开始就可以了
    std::vector<MapTwigWePtr> mLastSearchResult;

    /// 令k = mGatesMapping[j],
    /// 意味着mergedExpData->gates[j] 和 mFather->mergedExpData->gates[k]对应同一个Gate
    std::vector<size_t> mGatesMapping;

    /// 融合后的实际ExpData数据
    /// @note 如果是单个ExpData融合而成的, 这个ptr指向的就是原始的ExpDataPtr
    ExpDataPtr mergedExpData;

    size_t nMergedExps;

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
