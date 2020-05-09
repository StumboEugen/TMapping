//
// Created by stumbo on 2019/12/3.
//

#include "TopoMapping.h"
#include "MapTwig.h"
#include "tools/TopoParams.h"
#include "MergedExp.h"
#include "Exp.h"
#include "StructedMap.h"

#include <iostream>
#include <unordered_set>

using namespace std;

void tmap::TopoMapping::setLeftGate(GateID gateID)
{
    mExperiences.setLeftGateOfCurrent(gateID);
}

void tmap::TopoMapping::arriveNewExp(const tmap::ExpPtr& newExp)
{
#if TMAPPING_CONFIG_LOG_VERBOSE
    cout << "\n\n===============================\nSTART arrive new EXP" << endl;
#endif
    mExperiences.addNewExpAndAddLoopClosures(newExp, twigCollection);

    MergedExpPtr theSingleMergedExp = MergedExp::singleMergedFromExp(newExp);

    for (auto & oneAliveTwig : twigCollection.getAliveMaps()) {
        switch (oneAliveTwig->getStatus()) {
            case MapTwigStatus::EXPIRED:
                cerr << FILE_AND_LINE << " A expired twig in alive container!" << endl;
                break;
            case MapTwigStatus::MOVE2NEW:
                if (oneAliveTwig->hasChildren()) {
                    /// 多重的闭环结果已经在 mExperiences.addNewExpAndAddLoopClosures 完成
                    oneAliveTwig->setExpired();
                } else {
                    /// 该MapTwig继续存活, 认为newExp是独立的Exp
                    oneAliveTwig->nodeCountPlus();
                    oneAliveTwig->addMergedExp(theSingleMergedExp);
                    theSingleMergedExp->addRelatedMapTwig(oneAliveTwig);
                    twigCollection.add2NextGeneration(std::move(oneAliveTwig));
                }
                break;
            case MapTwigStatus::MOVE2OLD:
                const auto& oldSimiliarExp = oneAliveTwig->getTheArrivingSimiliarMergedExp();
                auto theNewMergedExpShouldBe = oldSimiliarExp->theNewestChild().lock();

                /// 下面这个if是为了确认对应的MergedExp是否已经生成了
                if (theNewMergedExpShouldBe &&
                    theNewMergedExpShouldBe->serialOfLastExp() == newExp->serial())
                {
                    /// 已经生成了, 检查一下gate的一致性问题(实际到达的gate是否能映射到历史上对应的gate)
                    if (theNewMergedExpShouldBe->gateMapping2Father(newExp->getEnterGate()) ==
                        oneAliveTwig->gateOfSimilarMergedExp())
                    {
                        /// 一致性通过, 直接相互建立引用
                        oneAliveTwig->addMergedExp(theNewMergedExpShouldBe);
                        theNewMergedExpShouldBe->addRelatedMapTwig(oneAliveTwig);
                        twigCollection.add2NextGeneration(std::move(oneAliveTwig));
                    } else {
                        /// 一致性不通过, 废弃
#if TMAPPING_CONFIG_LOG_VERBOSE
                        cout << "killed by OLD not alike (gened)" << endl;
#endif
                        oneAliveTwig->setExpired();
                    }
                } else {
                    /// 对应的MergedExp没有生成, 自己尝试生成, 先进行匹配
                    auto matchResult = oldSimiliarExp->detailedMatching(*newExp->expData());
                    double poss = matchResult->possibility;
                    bool gateCorrect = matchResult->gateMapping2old[newExp->getEnterGate()] ==
                                       oneAliveTwig->gateOfSimilarMergedExp();
                    if (gateCorrect && poss > TOLLERANCE_2ND_MATCH_MERGEDEXP) {
                        /// 一致性通过而且匹配概率也符合阈值, 生成对应的mergedExp
                        auto newMergedExp =
                                oldSimiliarExp->bornOne(
                                        newExp, std::move(matchResult), false);
                        oneAliveTwig->addMergedExp(newMergedExp);
                        newMergedExp->addRelatedMapTwig(oneAliveTwig);
                        twigCollection.add2NextGeneration(std::move(oneAliveTwig));
                    } else{
#if TMAPPING_CONFIG_LOG_VERBOSE
                        cout << "killed by OLD not alike (not gened) gateCor:"
                            << gateCorrect << " And the poss: " << poss << "the twig poss:"
                            << oneAliveTwig->getCaledGblConfidence() << endl;
#endif
                        oneAliveTwig->setExpired();
                    }
                }
        } /// switch end
    }

    if (newExp->serial() == 0) {
        auto adam = twigCollection.generateAdam();
        adam->addMergedExp(theSingleMergedExp);
        theSingleMergedExp->addRelatedMapTwig(adam);
    }

    twigCollection.nextgCompleteAdding(nSurviverMaps, newExp->serial() + 1);

    const auto& aliveTwigs = twigCollection.getAliveMaps();
    cout << "\n\nSteps:" << mExperiences.nExps()
       << "\nnumber of alive maps :" << twigCollection.getAliveMaps().size()
       << "\n------------------------" << endl;
#if TMAPPING_CONFIG_LOG_VERBOSE
    for (int i = 0; i < 10 && i < aliveTwigs.size(); ++i) {
        cout << "nNodes: " << aliveTwigs[i]->getNodeCount() << "\tpsbly: " <<
        twigCollection.getScores()[i] << endl;
    }
#endif

    auto& currentChampion = twigCollection.getAliveMaps().front();
    bool championBiggerThan90 = false;
    if (mChampionMap) {
        MapTwigPtr relatedTwig = mChampionMap->relatedTwig().lock();
        if (relatedTwig) {
            if (currentChampion->isDevelopedFrom(relatedTwig.get())) {
#if TMAPPING_CONFIG_LOG_VERBOSE
                cout << "[THE CHAMPION STATUS]   remains" << endl
#endif
                        ;
                if (twigCollection.getScores()[0] > 0.90) {
                    championBiggerThan90 = true;
                }
            } else {
#if TMAPPING_CONFIG_LOG_VERBOSE
                cout << "[THE CHAMPION STATUS]   changed" << endl;
#endif
            }
        } else {
#if TMAPPING_CONFIG_LOG_VERBOSE
            cout << "[THE CHAMPION STATUS]   is killed" << endl;
#endif
        }
    }
    if (championBiggerThan90) {
        nChampionDefend++;
    } else {
        nChampionDefend = 0;
    }

    mChampionMap = currentChampion->makeMap(this->mExperiences);
}

tmap::Jsobj tmap::TopoMapping::getTopMaps(size_t nTops)
{
    Jsobj res;

    if (nTops == 0) {
        nTops = SIZE_MAX;
    }
    size_t nPushed = 0;

    const auto& aliveMaps = twigCollection.getAliveMaps();
    if (aliveMaps.empty()) {
        cerr << FILE_AND_LINE << " You ask maps from an empty twigCollection!" << endl;
        return res;
    }

    nTops = min(nTops, aliveMaps.size());

    for (int i = 0; i < nTops; ++i) {
        const auto& structedMap = aliveMaps[i]->makeMap(this->mExperiences);
        structedMap->setPsblt(this->twigCollection.getScores()[i]);
        res["maps"].append(structedMap->toJS());
    }

#ifdef TMAPPING_CONFIG_RECORD_POSS
    for (const auto& poss : twigCollection.getChampionsPoss()) {
        res["championsPoss"].append(poss);
    }
    for (const auto& poss : twigCollection.getRunnerUpsPoss()) {
        res["runnerUpsPoss"].append(poss);
    }
#endif

    return res;
}

void tmap::TopoMapping::setNSurviverMaps(size_t nMaps)
{
    TopoMapping::nSurviverMaps = nMaps;
}

size_t tmap::TopoMapping::getChampionDefendedCount() const
{
    return nChampionDefend;
}

tmap::Jsobj tmap::TopoMapping::getChampionHistory()
{
    return twigCollection.getAliveMaps().front()->structedMapHistory(mExperiences);
}
