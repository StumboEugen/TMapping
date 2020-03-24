//
// Created by stumbo on 2019/11/15.
//

#include "ExpCollection.h"
#include "Exp.h"
#include "MergedExp.h"
#include "MapTwig.h"
#include "MapTwigCollection.h"

void tmap::ExpCollection::setLeftGateOfCurrent(GateID leftGate)
{
    const auto& backExp = mExperiencesData.back();
    /// 遍历尾端Exp的所有mergedExp使用情况, 查找是否发生移动到已通过点的情况
    for (const auto & mergedExpWeak : backExp->getMergedExps()) {
        if (!mergedExpWeak.expired()) {
            auto mergedExp = mergedExpWeak.lock();
            auto gatesOccRes = mergedExp->checkGateConflict(leftGate);
            if (gatesOccRes.conflictExp != nullptr) {
                /// 发生gate的冲突, 找到相连的其他Exp以及对应gate, 用于设置mergedExp相关Map的similarNext
                const auto& similarExp = mExperiencesData.at(
                        gatesOccRes.conflictExp->serial() + (gatesOccRes.enter ? -1 : 1));
                GateID theGoingRelatedGate = gatesOccRes.enter ?
                                             similarExp->getLeaveGate() :
                                             similarExp->getEnterGate();
                mergedExp->setRelatedTwigsNextMove2old(similarExp, theGoingRelatedGate);
            } else {
                /// 没有发生, 说明正在移动前往一个新的Exp
                mergedExp->setRelatedTwigsNextMove2new();
            }
        }
    }
    backExp->setLeftGate(leftGate);
}

void tmap::ExpCollection::setLeftGateOfCurrent(const TopoVec2& gatePos)
{
    GateID cloestGate = mExperiencesData.back()->expData()->findTheCloestGate(gatePos);
    setLeftGateOfCurrent(cloestGate);
}

void tmap::ExpCollection::addNewExpAndAddLoopClosures(tmap::ExpPtr newExp,
                                                      tmap::MapTwigCollection& twigMaster)
{
    newExp->setSerial(mExperiencesData.size());
    newExp->setOdomInfoFromFatherExp(mExperiencesData.back());

    auto & vecSameType = mClassification[newExp->expData()->type()];

    for (auto& sameTypeExp : vecSameType) {
        if (!sameTypeExp->expData()->quickMatch(*newExp->expData(), 1)) {
            continue;
        }

        /// 需要确保这个mergedExp在当前函数里保持存活
        MergedExpPtr currentSingleMergedExp = MergedExp::singleMergedFromExp(newExp);

        size_t expiredCount = 0;
        auto& mergedExps = sameTypeExp->getMergedExps();
        for (const auto& mergedExpWe : mergedExps) {
            auto mergedExp = mergedExpWe.lock();
            if (mergedExp) {
                auto currentMatchResult = mergedExp->detailedMatching(*newExp->expData());
                auto poss2 = currentMatchResult->possibility;
                /// 相似概率要高于一定的阈值
                if (poss2 > TOLLERANCE_2ND_MATCH_MERGEDEXP)
                {
                    auto conflictRes = mergedExp->checkGateConflict
                            (currentMatchResult->gateMapping2old[newExp->getEnterGate()]);
                    /// 而且这个入口不能和之前的出入口发生冲突, 闭环必须发生在没有走过的gate上

                    if (conflictRes.conflictExp == nullptr)
                    {
                        /// 并没有发生gate上的冲突, 于是开始找使用这个MergedExp的末端MapTwig
                        auto closureTwigs = mergedExp->findTwigsUsingThis();
                        if (!closureTwigs.empty())
                        {
                            auto newMergedExp = mergedExp->bornOne(
                                    newExp, std::move(currentMatchResult));
                            newMergedExp->reserveTwigs(closureTwigs.size() + 1);
                            /// 为每个使用mergedExp的Twig构造形成实质闭环的Twig后代
                            for (auto& twig2born : closureTwigs)
                            {
                                if (!twig2born->hasChildren())
                                {
                                    /// 这是第一个分叉, 除了闭环之外还要负责生成always new的假设
                                    twig2born->setDieAt(newExp->serial());
                                    /// 这里产生后代后, father的status不会变化,
                                    /// 从而不会影响其他mergedExp对MapTwig的搜索 findTwigsUsingThis()
                                    auto newTwigAssumingNew = twigMaster.bornOne(twig2born,
                                            currentSingleMergedExp);
                                    newTwigAssumingNew->nodeCountPlus();
                                }
                                auto twigWithClosure = twigMaster.bornOne(twig2born,
                                                                          newMergedExp);
                            }
                        }
                    }
                }
            } else {
                expiredCount++;
            }
        }

        if (expiredCount > mergedExps.size() / 2) {
            sameTypeExp->cleanUpExpiredMergedExps();
        }
    }

    vecSameType.push_back(newExp.get());
    mExperiencesData.push_back(std::move(newExp));
}

const tmap::ExpPtr& tmap::ExpCollection::getExpAt(size_t serial) const
{
    return mExperiencesData[serial];
}
