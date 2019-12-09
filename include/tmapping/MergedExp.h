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

class MergedExp : public std::enable_shared_from_this<MergedExp>
{
    /// 记录从哪个mergedExp分裂而来的, 可能为null
    MergedExpPtr mFather;

    /// 在此mergedExp新加入的Exp
    const ExpPtr mRelatedExp;

    /// 使用这个MergedExp的Twig们(可能有的已经被废弃了)
    std::vector<MapTwigWePtr> mRelatedMaps{};

    /// 融合后的实际ExpData数据
    /// @note 如果是单个ExpData融合而成的, 这个ptr指向的就是原始的ExpDataPtr
    ExpDataPtr mMergedExpData;

    /// 这个融合体是多少Exp融合而成的
    const size_t nMergedExps;

    /// mFather->mMergedExpData相对于this->mMergedExpData的位移
    TopoVec2 mTrans;

    /// 令k = mGatesMapping[j],
    /// 意味着mergedExpData->gates[j] 和 mFather->mergedExpData->gates[k]对应同一个Gate.
    /// 如果 mFather == nullptr, 则该成员为空
    std::vector<size_t> mGatesMapping;

    /// 这次融合的概率降低系数
    double mPossDecConf;

    /// 最近一次分裂出的后代, 为了便于MOVE2OLD
    MergedExpWePtr mNewestChild{};

    /// 构造的时候不会把LastSearchResult用RelatedMaps填满,
    /// 因此需要这个bool来判断 mLastSearchResult空的时候代表什么
    bool hasSearchedLoopClosure = false;
    /// 每次的搜索不需要从头开始, 从上次的结果开始就可以了
    std::vector<MapTwigWePtr> mLastSearchResult{};

private: // constructor

    MergedExp(MergedExpPtr father, ExpPtr newExp, MatchResult matchResult);

    explicit MergedExp(ExpPtr fatherExp);

public:

    /**
     * @brief 针对实际数据进行匹配
     * @see MatchResult_IMPL
     */
    MatchResult detailedMatching(const MergedExp& another) const;

    /**
     * @brief 针对实际数据进行匹配
     * @see MatchResult_IMPL
     */
    MatchResult detailedMatching(const ExpData& expData) const;

    size_t serialOfLastExp() const;

    bool isChildOf(const MergedExp* possibileFather) const;

    /**
     * @brief 找到使用这个MergedExp的所有MOVE2NEW末端Twig
     * @return 与此MergedExp可能的闭环的末端MapTwig
     */
    std::vector<MapTwigPtr> findTwigsUsingThis();

    /**
     * @brief 生成一个和this MergedExp有关的后代Exp
     * @param newExp 需要额外融合的新Exp
     * @param matchResult 上次匹配的结果, 包含有融合后的ExpData信息以及gate对应关系
     * @return 新的MergedExp
     */
    MergedExpPtr bornOne(ExpPtr newExp, MatchResult matchResult);

    /**
     * @brief 从一个ExpData生成独立的single MergedExp.
     * 这样的MergedExp算不上MergedExp,
     * 而且并不是每一次都生成一个新的, 如果fatherExp中已经存储了一个, 会用同一个
     * @param fatherExp
     * @return
     */
    static MergedExpPtr bornFromExp(ExpPtr fatherExp);

    void addRelatedMapTwig(const MapTwigPtr& twigPtr);

    /**
     * @brief mRelatedTwigs.reserve(n)
     * 需要添加MapTwig时往往对应的数量已经知道了,
     */
    void reserveTwigs(size_t n);

    /// 获取最新的child, 从而方便MOVE2OLD的MapTwig找到对应的MergedExp
    const MergedExpWePtr& theNewestChild() const;

    double getPossDecConf() const;

    /**
     * @brief 检查指定的Gate是否已经被作为入口或者出口过了
     * @param gateID 检查的gateID, 相对于this而言
     * @return true表示被占用, false表示没有被占用
     */
    bool checkIfGateIsOccupied(size_t gateID);
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MERGEDEXP_H
