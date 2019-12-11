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
                        oneAliveTwig->setExpired();
                    }
                } else {
                    /// 对应的MergedExp没有生成, 自己尝试生成, 先进行匹配
                    auto matchResult = oldSimiliarExp->detailedMatching(*newExp->expData());
                    double poss = matchResult->possibility;
                    bool gateCorrect = matchResult->gateMapping2this[newExp->getEnterGate()] ==
                            oneAliveTwig->gateOfSimilarMergedExp();
                    if (gateCorrect && poss > TOLLERANCE_2ND_MATCH_MERGEDEXP) {
                        /// 一致性通过而且匹配概率也符合阈值, 生成对应的mergedExp
                        auto newMergedExp =
                                oldSimiliarExp->bornOne(newExp, std::move(matchResult));
                        oneAliveTwig->addMergedExp(newMergedExp);
                        newMergedExp->addRelatedMapTwig(oneAliveTwig);
                        twigCollection.add2NextGeneration(std::move(oneAliveTwig));
                    } else{
                        oneAliveTwig->setExpired();
                    }
                }
        } /// switch end
    }

    if (newExp->serial() == 0) {
        auto adam = twigCollection.generateAdam();
        adam->addMergedExp(theSingleMergedExp);
        theSingleMergedExp->addRelatedMapTwig(adam);
        twigCollection.add2NextGeneration(std::move(adam));
    }

    twigCollection.nextgCompleteAdding(mSurviverSetting, newExp->serial() + 1);

    /// TODO 构建可理解拓扑地图的工作
}
