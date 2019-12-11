//
// Created by stumbo on 2019/11/15.
//

#include "ExpCollection.h"
#include "Exp.h"
#include "MergedExp.h"
#include "MapTwig.h"

void tmap::ExpCollection::setLeftGateOfCurrent(size_t leftGate)
{
    const auto& backExp = mExperiencesData.back();
    for (const auto & mergedExpWeak : backExp->getMergedExps()) {
        if (!mergedExpWeak.expired()) {
            auto mergedExp = mergedExpWeak.lock();
            auto gatesOccRes = mergedExp->checkGateConflict(leftGate);
            if (gatesOccRes.conflictExp != nullptr) {
                const auto& similarExp = mExperiencesData.at(
                        gatesOccRes.conflictExp->serial() + (gatesOccRes.enter ? -1 : 1));
                size_t theGoingRelatedGate = gatesOccRes.enter ?
                                             similarExp->getLeftGate() :
                                             similarExp->getEnterGate();
                mergedExp->setRelatedTwigsNextMove2old(similarExp, theGoingRelatedGate);
            } else {
                mergedExp->setRelatedTwigsNextMove2new();
            }
        }
    }
    backExp->setLeftGate(leftGate);
}

void tmap::ExpCollection::setLeftGateOfCurrent(const TopoVec2& gatePos)
{
    auto cloestGate = mExperiencesData.back()->expData()->findTheCloestGate(gatePos);
    setLeftGateOfCurrent(cloestGate);
}

void tmap::ExpCollection::addNewExpAndAddLoopClosures(tmap::ExpPtr newExp,
                                                      tmap::MapTwigCollection& twigMaster)
{
    newExp->setSerial(mExperiencesData.size());

    auto & vecSameType = mClassification[newExp->expData()->type()];

    for (size_t i = vecSameType.size() - 1; i >= 0; ++i) {
        Exp* sameTypeExp = vecSameType[i];
        double poss1 = sameTypeExp->expData()->quickMatch(*newExp->expData(), 1);
        if (poss1 < TOLLERANCE_1ST_MATCH_EXP) {
            continue;
        }

        /// 需要确保这个mergedExp在当前函数里保持存活
        MergedExpPtr currentSingleMergedExp = MergedExp::bornFromExp(newExp);

        size_t expiredCount = 0;
        auto& mergedExps = sameTypeExp->getMergedExps();
        for (const auto& iter : mergedExps) {
            auto mergedExp = iter.lock();
            if (mergedExp) {
                auto currentMatchResult = mergedExp->detailedMatching(*newExp->expData());
                auto poss2 = currentMatchResult->possibility;
                /// 相似概率要高于一定的阈值
                if (poss2 > TOLLERANCE_2ND_MATCH_MERGEDEXP)
                {
                    auto conflictRes = mergedExp->checkGateConflict
                            (currentMatchResult->gateMapping2this[newExp->getEnterGate()]);
                    /// 而且这个入口不能和之前的出入口发生冲突, 闭环必须发生在没有走过的gate上

                    if (conflictRes.conflictExp != nullptr)
                    {
                        /// 并没有发生gate上的冲突, 于是开始找使用这个MergedExp的末端MapTwig
                        auto closureTwigs = mergedExp->findTwigsUsingThis();
                        if (!closureTwigs.empty())
                        {
                            auto newMergedExp = mergedExp->bornOne(
                                    newExp, std::move(currentMatchResult));
                            newMergedExp->reserveTwigs(closureTwigs.size() + 1);
                            for (auto& twig2born : closureTwigs)
                            {
                                if (!twig2born->hasChildren())
                                {
                                    /// 这是第一个分叉, 除了本闭环之外还要负责生成always new
                                    twig2born->setDieAt(newExp->serial());
                                    /// 这里产生后代后, father的status不会变化, 从而不会影响其他mergedExp对MapTwig的搜索
                                    auto newTwigAssumingNew = twigMaster.bornOne(twig2born,
                                            currentSingleMergedExp);
                                    newTwigAssumingNew->nodeCountPlus();
                                    currentSingleMergedExp->addRelatedMapTwig(
                                            newTwigAssumingNew);
                                }
                                auto twigWithClosure = twigMaster.bornOne(twig2born,
                                                                          newMergedExp);
                                newMergedExp->addRelatedMapTwig(twigWithClosure);
                            }
                        }
                    }
                }
            } else {
                expiredCount++;
            }
        }

        if (expiredCount > mergedExps.size() / 2) {
            sameTypeExp->cleanUpMergedExps();
        }
    }

    vecSameType.push_back(newExp.get());
    mExperiencesData.push_back(std::move(newExp));
}
