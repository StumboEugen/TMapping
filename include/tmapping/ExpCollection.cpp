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
        auto currentSingleMergedExp = MergedExp::bornFromExp(newExp);
        newExp->theSingleMergedExp() = currentSingleMergedExp;

        //TODO 不要忘了单独闭环的情况
        auto& mergedExps = sameTypeExp->getMergedExps();
        for (const auto& iter : mergedExps) {
            auto mergedExp = iter.lock();
            if (mergedExp) {
                auto currentMatchResult = mergedExp->detailedMatching(*newExp->expData());
                auto poss2 = currentMatchResult->possibility;
                if (poss2 > TOLLERANCE_2ND_MATCH_MERGEDEXP) {
                    /// VERY IMPORTANT PART
                    auto closureTwigs = mergedExp->getMostRecentLoopClosureMaps();
                    if (!closureTwigs.empty()) {
                        auto newMergedExp = mergedExp->bornOne(
                                newExp, std::move(currentMatchResult));
                        newExp->addMergedIns(newMergedExp);
                        newMergedExp->reserveTwigs(closureTwigs.size());
                        for (auto & twig2born : closureTwigs) {
                            if (!twig2born->hasChildren()) {
                                /// 这是第一个分叉, 除了本闭环之外还要负责生成always new
                                twig2born->setDieAt(newExp->serial());
                                /// 这里产生后代后, father的status不会变化, 从而不会影响其他mergedExp对MapTwig的搜索
                                auto newTwigAssumingNew = twigMaster.bornOne(twig2born, 1.0);
                                newTwigAssumingNew->nodeCountPlus();
                                newTwigAssumingNew->addMergedExp(currentSingleMergedExp);
                                currentSingleMergedExp->addRelatedMapTwig(newTwigAssumingNew);
                            }
                            auto twigWithClosure = twigMaster.bornOne(twig2born, poss2);
                            twigWithClosure->addMergedExp(newMergedExp);
                            newMergedExp->addRelatedMapTwig(twigWithClosure);
                        }
                    }
                }
            } /// TODO 根据没用的数量来判断要不要重新做这个表
        }

        sameTypeExp->cleanUpMergedExps();
    }

    mExperiencesData.push_back(std::move(newExp));
}
