//
// Created by stumbo on 2019/11/15.
//

#include "ExpCollection.h"
#include "Exp.h"
#include "MergedExp.h"
#include "MapTwig.h"

void tmap::ExpCollection::setLeftGateOfCurrent(size_t leftGate)
{
    mExperiencesData.back()->setLeftGate(leftGate);
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
                if ( /// 相似概率要高于一定的阈值
                    poss2 > TOLLERANCE_2ND_MATCH_MERGEDEXP
                &&   /// 而且这个入口没有被之前的出入口占用, 闭环必须发生在没有走过的gate
                    !mergedExp->checkIfGateIsOccupied
                    (currentMatchResult->gateMapping2this[newExp->getEnterGate()]) )
                {
                    /// VERY IMPORTANT PART
                    auto closureTwigs = mergedExp->getMostRecentLoopClosureMaps();
                    if (!closureTwigs.empty()) {
                        auto newMergedExp = mergedExp->bornOne(
                                newExp, std::move(currentMatchResult));
                        newExp->addMergedExpIns(newMergedExp);
                        newMergedExp->reserveTwigs(closureTwigs.size());
                        for (auto & twig2born : closureTwigs) {
                            if (!twig2born->hasChildren()) {
                                /// 这是第一个分叉, 除了本闭环之外还要负责生成always new
                                twig2born->setDieAt(newExp->serial());
                                /// 这里产生后代后, father的status不会变化, 从而不会影响其他mergedExp对MapTwig的搜索
                                auto newTwigAssumingNew = twigMaster.bornOne(twig2born,
                                                                             currentSingleMergedExp);
                                newTwigAssumingNew->nodeCountPlus();
                                currentSingleMergedExp->addRelatedMapTwig(newTwigAssumingNew);
                            }
                            auto twigWithClosure = twigMaster.bornOne(twig2born,
                                                                      newMergedExp);
                            newMergedExp->addRelatedMapTwig(twigWithClosure);
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
