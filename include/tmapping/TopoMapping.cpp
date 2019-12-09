//
// Created by stumbo on 2019/12/3.
//

#include "TopoMapping.h"
#include "MapTwig.h"
#include "tools/TopoParams.h"
#include "MergedExp.h"

#include <iostream>
#include <unordered_set>

using namespace std;

void tmap::TopoMapping::addNewExp(tmap::ExpDataPtr newExpData)
{

}

void tmap::TopoMapping::setLeftGate(size_t gateID)
{
    mExperiences.setLeftGateOfCurrent(gateID);
}

void tmap::TopoMapping::setLeftGate(TopoVec2 gatePos)
{
    mExperiences.setLeftGateOfCurrent(gatePos);
}

void tmap::TopoMapping::arriveNewExp(const tmap::ExpPtr& newExp)
{
    mExperiences.addNewExpAndAddLoopClosures(newExp, twigCollection);

    MergedExpPtr theSingleMergedExp = MergedExp::bornFromExp(newExp);

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
                auto theShouldBeMergedPtr = oldSimiliarExp->theNewestChild().lock();
                if (theShouldBeMergedPtr &&
                        theShouldBeMergedPtr->serialOfLastExp() == newExp->serial()) {
                    oneAliveTwig->addMergedExp(theShouldBeMergedPtr);
                    //TODO BUG POSS NOT X
                    theShouldBeMergedPtr->addRelatedMapTwig(oneAliveTwig);
                } else {
                    auto matchResult = oldSimiliarExp->detailedMatching(*newExp->expData());
                    double poss = matchResult->possibility;
                    if (poss > TOLLERANCE_2ND_MATCH_MERGEDEXP) {
                        auto newMergedExp =
                                theShouldBeMergedPtr->bornOne(newExp, std::move(matchResult));
                        newExp->addMergedExpIns(newMergedExp);
                        oneAliveTwig->addMergedExp(newMergedExp);
                        newMergedExp->addRelatedMapTwig(oneAliveTwig);
                        twigCollection.add2NextGeneration(std::move(oneAliveTwig));
                    } else{
                        oneAliveTwig->setExpired();
                    }
                }
        }
    }

    if (newExp->serial() == 0) {
        auto adam = twigCollection.generateAdam();
        adam->addMergedExp(theSingleMergedExp);
        theSingleMergedExp->addRelatedMapTwig(adam);
        twigCollection.add2NextGeneration(std::move(adam));
    }

    twigCollection.nextgCompleteAdding(mSurviverSetting, newExp->serial() + 1);
}
