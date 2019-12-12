//
// Created by stumbo on 2019/11/17.
//

#include "MapTwig.h"
#include "Tmapping.h"
#include "MergedExp.h"
#include "Exp.h"
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

void MapTwig::setTheSimilarMergedExpForNextTime(const ExpPtr& targetExp, size_t arrivingGate)
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
                /// TODO 这里会平白无故增加工作量, 未来可能考虑匹配结果要存储一张双向的映射表
                arrivingGate = mergedExp->findReverseGateMapping(arrivingGate);
                theSimilarMergedExp = mergedExp;
            }
        }
    }
    mExpOfArrivingSimilar = theSimilarMergedExp;
    mGateOfSimilar = arrivingGate;
}

size_t MapTwig::gateOfSimilarMergedExp() const
{
    return mGateOfSimilar;
}

void MapTwig::setMove2new()
{
    status = MapTwigStatus::MOVE2NEW;
}
