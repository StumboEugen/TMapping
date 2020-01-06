//
// Created by stumbo on 2019/11/17.
//

#include "MapTwig.h"
#include "Tmapping.h"
#include "MergedExp.h"
#include "Exp.h"
#include "StructedMap.h"
#include <iostream>
#include <cmath>

using namespace std;
using namespace tmap;

tmap::MapTwigPtr tmap::MapTwig::getAdamTwig()
{
    MapTwigPtr adam(new MapTwig(0, nullptr, 0, 1.0));
    return adam;
}

tmap::MapTwig::MapTwig(size_t bornAt, MapTwigPtr father,
                       size_t nSerial, double confidence) : borndAt(bornAt),
                                                            mFather(std::move(father)),
                                                            nSerial(nSerial),
                                                            mConfidence(confidence)
{
    if (father) {
        nTopoNode = father->nTopoNode;
    } else {
        nTopoNode = 1;
    }
}

tmap::MapTwigPtr tmap::MapTwig::bornOne(size_t newSerial)
{
    MapTwigPtr newTwig(new MapTwig(
            this->mDieAt,
            shared_from_this(),
            newSerial,
            this->mConfidence));
    /// 在后代列表后插入这个新的后代
    this->mChildren.emplace_back(newTwig);
    return newTwig;
}

void tmap::MapTwig::setExpired()
{
    if (status == MapTwigStatus::EXPIRED) {
        cerr << FILE_AND_LINE << " You expired a mapTwig more than once!" << endl;
    }
    status = MapTwigStatus::EXPIRED;
}

tmap::MapTwigStatus tmap::MapTwig::getStatus() const
{
    return status;
}

const tmap::MergedExpPtr& tmap::MapTwig::getTheArrivingSimiliarMergedExp() const
{
    return mExpOfArrivingSimilar;
}

double tmap::MapTwig::xConfidenceCoe(double coe)
{
    mConfidence *= coe;
    return mConfidence;
}

const vector<tmap::MapTwigWePtr>& tmap::MapTwig::getChildren() const
{
    return mChildren;
}

const vector<tmap::MergedExpPtr>& tmap::MapTwig::getExpUsages() const
{
    return mExpUsages;
}

bool tmap::MapTwig::hasChildren() const
{
    return !mChildren.empty();
}

double tmap::MapTwig::getConfidence() const
{
    return mConfidence;
}

void tmap::MapTwig::nodeCountPlus()
{
    nTopoNode++;
}

void tmap::MapTwig::setDieAt(size_t dieAt)
{
    mDieAt = dieAt;
}

void tmap::MapTwig::addMergedExp(tmap::MergedExpPtr newMerged)
{
    xConfidenceCoe(newMerged->getPossDecConf());
    mExpUsages.emplace_back(std::move(newMerged));
}

void tmap::MapTwig::resetLastGlobalConfidenceResult()
{
    mLastGlobalResult = -1.0;
}

double tmap::MapTwig::calGlobalPoss(double log_nExp)
{
    if (mLastGlobalResult == -1.0) {
        mLastGlobalResult = mConfidence * exp( -nTopoNode * log_nExp);
    }
    return mLastGlobalResult;
}

void MapTwig::setTheSimilarMergedExpForNextTime(const ExpPtr& targetExp, GateID arrivingGate)
{
    status = MapTwigStatus::MOVE2OLD;

    /// 构建从this到targetExp对应的MapTwig的链条
    vector<MapTwig*> chainToFather;
    MapTwig* current = this;
    std::size_t targetSerial = targetExp->serial();
    while (current->borndAt > targetSerial) {
        chainToFather.push_back(current);
        current = current->mFather.get();
    }
    chainToFather.push_back(current);
    /// 构建完毕

    /// 从过去往现在查找, 找到对应的Exp最新的MergedExp, 并记录下来
    auto& theSimilarMergedExp = current->mExpUsages.at(targetSerial - current->borndAt);
    for (auto iter = chainToFather.rbegin(); iter != chainToFather.rend(); ++iter) {
        for (const auto& mergedExp : (*iter)->mExpUsages) {
            if (mergedExp->isChildOf(theSimilarMergedExp.get())) {
                arrivingGate = mergedExp->mapGateFromFather(arrivingGate);
                theSimilarMergedExp = mergedExp;
            }
        }
    }
    mExpOfArrivingSimilar = theSimilarMergedExp;
    mGateOfSimilar = arrivingGate;
}

GateID MapTwig::gateOfSimilarMergedExp() const
{
    return mGateOfSimilar;
}

void MapTwig::setMove2new()
{
    status = MapTwigStatus::MOVE2NEW;
}

StructedMap MapTwig::makeMap(const ExpCollection& exps)
{
    if (status == MapTwigStatus::EXPIRED) {
        cerr << FILE_AND_LINE << " You are making a map from an expired twig!" << endl;
    }
    
    struct __realGateIndex{
        GateID enterGate = GATEID_BEGINNING_POINT;
        GateID leaveGate = GATEID_HAVENT_LEFT;
    };

    auto expSize = mExpUsages.back()->serialOfLastExp();
    /// 记录每个Exp对应的MapNode(MergedExp)是哪一个
    vector<MapNodePtr> nodePs(expSize);
    /// 记录每个Exp的出入gate在其对应的MergedExp的实际Gate编号
    vector<__realGateIndex> realGates(expSize);
    /// 实际的几个MapNode
    vector<MapNodePtr> mapNodePtrs;
    mapNodePtrs.reserve(expSize);

    /// 从current MapTwig开始遍历, 直到最初的MapTwig
    const MapTwig* currentTwig = this;
    while (currentTwig != nullptr) {
        for (auto iter = mExpUsages.rbegin(); iter != mExpUsages.rend(); ++iter) {
            /// ExpUsages也是从后向前遍历
            auto& mergedExp = *iter;
            size_t currentSerial = mergedExp->serialOfLastExp();
            auto& relatedNodePtr = nodePs[currentSerial];
            if (relatedNodePtr == nullptr) {
                /// 如果对应的MapNode不存在,说明这个mergedExp是第一次被加入,也对应MapNode的实际related Exp
                relatedNodePtr = MapNode::makeOneFromMergedExp(mergedExp);
                const auto& currentMergedRelatedExp = mergedExp->getTheLastExp();
                mapNodePtrs.emplace_back(relatedNodePtr);

                size_t nGates = currentMergedRelatedExp->expData()->nGates();
                vector<GateID> gatesMap2EndMergedExp(nGates);
                for (int i = 0; i < nGates; ++i) {
                    gatesMap2EndMergedExp[i] = i;
                }
                GateID enterGate = currentMergedRelatedExp->getEnterGate();
                GateID leaveGate = currentMergedRelatedExp->getLeaveGate();
                if (enterGate != GATEID_BEGINNING_POINT) {
                    realGates[currentSerial].enterGate = gatesMap2EndMergedExp[enterGate];
                }
                if (leaveGate != GATEID_HAVENT_LEFT) {
                    realGates[currentSerial].leaveGate = gatesMap2EndMergedExp[leaveGate];
                }

                MergedExp* fatherMergedExp = mergedExp->getFather().get();
                if (fatherMergedExp != nullptr) {
                    mergedExp->mapGates(gatesMap2EndMergedExp);

                    /// 让 mergedExp 的所有 father 对应序号的 NodeP 都指向刚刚由 mergedExp 生成的 relatedNodePtr
                    while (fatherMergedExp != nullptr) {
                        size_t oneFatherSerial = fatherMergedExp->getTheLastExp()->serial();
                        nodePs[oneFatherSerial] = relatedNodePtr;

                        /// 记录这个位置的gate真实映射关系
                        leaveGate = fatherMergedExp->getTheLastExp()->getLeaveGate();
                        realGates[oneFatherSerial].leaveGate = gatesMap2EndMergedExp[leaveGate];

                        enterGate = fatherMergedExp->getTheLastExp()->getEnterGate();
                        if (enterGate != GATEID_BEGINNING_POINT) {
                            realGates[oneFatherSerial].enterGate = gatesMap2EndMergedExp[enterGate];
                        }
                        fatherMergedExp->mapGates(gatesMap2EndMergedExp);

                        /// 下一个father
                        fatherMergedExp = fatherMergedExp->getFather().get();
                    }
                }
            } else {
                /// 对应的Node已经被生成过了, 说明current mergedExp的child在这个Map的MapNode里,
                /// 换句话说, current对于this map而言是不完整的
                continue;
            }
        }
        currentTwig = currentTwig->mFather.get();
    }
    
    /// DEBUG TEST: 检查nodePsVec是否都被充满了
    for (const auto& nodeP : nodePs) {
        if (nodeP == nullptr) {
            cerr << FILE_AND_LINE << "ERROR! every exp should find a mapNode related" << endl;
            throw;
        }
    }

    /// 沿着Experience设定每个MapNode的连接关系
    for (int i = 0; i < expSize - 1; ++i) {
        auto leaveGate = realGates[i].leaveGate;
        auto enterGate = realGates[i + 1].enterGate;
        auto& leaveLink = nodePs[i]->linkAt(leaveGate);
        auto& enterLink = nodePs[i + 1]->linkAt(enterGate);
        leaveLink.to = nodePs[i + 1];
        enterLink.to = nodePs[i];
        leaveLink.at = enterGate;
        enterLink.at = leaveGate;
    }

    auto structedMap = make_shared<StructedMapImpl>(
            mapNodePtrs, shared_from_this(), mLastGlobalResult);

    return structedMap;
}

bool MapTwig::isDevelopedFrom(MapTwig* twig2check, size_t nGenerations) const
{
    if (nGenerations == 0) {
        nGenerations = SIZE_MAX;
    }
    size_t nGens = 0;
    const MapTwig* familyMember = this;
    while (familyMember != nullptr && nGens++ <= nGenerations) {
        if (twig2check == familyMember) {
            return true;
        }
        familyMember = familyMember->mFather.get();
    }
    return false;
}
