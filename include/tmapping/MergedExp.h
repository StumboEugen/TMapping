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
    /// 记录从哪个mergedExp分裂而来的, 可能为null
    MergedExpPtr mFather;

    /// 在此mergedExp新加入的Exp
    const ExpPtr mRelatedExp;

    /// 使用这个MergedExp的Twig们(可能有的已经被废弃了)
    std::vector<MapTwigWePtr> mRelatedMaps;

    /// 这个融合体是多少Exp融合而成的
    size_t nMergedExps;

    /// 构造的时候不会把LastSearchResult用RelatedMaps填满, 因此需要这个bool来判断last result空的时候代表什么
    bool hasSearchedLoopClosure = false;

    /// 每次的搜索不需要从头开始, 从上次的结果开始就可以了
    std::vector<MapTwigWePtr> mLastSearchResult;

    /// 令k = mGatesMapping[j],
    /// 意味着mergedExpData->gates[j] 和 mFather->mergedExpData->gates[k]对应同一个Gate
    std::vector<size_t> mGatesMapping;

    /// mFather->mMergedExpData相对于this->mMergedExpData的位移
    TopoVec2 mTrans;

    /// 融合后的实际ExpData数据
    /// @note 如果是单个ExpData融合而成的, 这个ptr指向的就是原始的ExpDataPtr
    ExpDataPtr mMergedExpData;

public:
    MatchResult detailedMatching(const MergedExp& another) const;

    MatchResult detailedMatching(const ExpData& expData) const;

    size_t lastExpSerial() const;

    bool isLastButOneExpSerial(size_t serial2Check) const;

    /**
     * @brief 找到MOVE2NEW的末端Twig
     * @return 与此MergedExp可能的闭环的末端MapTwig
     */
    std::vector<MapTwigPtr> getLoopClosureMaps();

    MergedExp(MergedExpPtr father, ExpPtr newExp, MatchResult matchResult);

    MergedExp(ExpPtr fatherExp);
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
