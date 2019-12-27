//
// Created by stumbo on 2019/12/2.
//

#include "MergedExp.h"
#include "Exp.h"
#include "MapTwig.h"

#include <iostream>

using namespace std;
using namespace tmap;

MergedExp::MergedExp(MergedExpPtr father, ExpPtr newExp, MatchResult matchResult)
        : mFather(std::move(father)),
          mRelatedExp(std::move(newExp)),
          nMergedExps(father->nMergedExps + 1),
          mMergedExpData(std::move(matchResult->mergedExpData)),
          mTrans(matchResult->displacement),
          mGatesMapping2Father(std::move(matchResult->gateMapping2this)),
          mGatesMappingFromFather(std::move(matchResult->gateMapping2mergedExpData)),
          mPossDecConf(matchResult->possibility)
{}

MergedExp::MergedExp(ExpPtr fatherExp)
        : mFather(nullptr),
          mRelatedExp(std::move(fatherExp)),
          nMergedExps(1),
          mMergedExpData(mRelatedExp->expData()),
          mTrans(),
          /// add GATEID_NO_MAPPING for safety
          mGatesMapping2Father(mMergedExpData->nGates(), GATEID_NO_MAPPING),
          mGatesMappingFromFather(),
          mPossDecConf(1.0)
{}

MergedExp::MergedExp(const Jsobj& jmergedExp)
        : mFather(),
          mRelatedExp(),
          nMergedExps(jmergedExp["exps"].size()),
          mMergedExpData(ExpData::madeFromJS(jmergedExp["data"])),
          mTrans(),
          mGatesMapping2Father(),
          mGatesMappingFromFather(),
          mPossDecConf(1.0)
{

}

MatchResult tmap::MergedExp::detailedMatching(const tmap::MergedExp& another) const
{
    double weight = this->nMergedExps;
    weight /= another.nMergedExps;
    return mMergedExpData->detailedMatch(*another.mMergedExpData, weight);
}

MatchResult tmap::MergedExp::detailedMatching(const ExpData& expData) const
{
    return mMergedExpData->detailedMatch(expData, this->nMergedExps);
}

size_t tmap::MergedExp::serialOfLastExp() const
{
    return mRelatedExp->serial();
}

bool MergedExp::isChildOf(const MergedExp* possibileFather) const
{
    return possibileFather == mFather.get();
}

static void TOOLFUN_extractSharedFromWeak(vector<MapTwigPtr>& to, vector<MapTwigWePtr>& from) {
    to.reserve(from.size() * 2);
    for (const auto & oldWePtr: from) {
        auto shPtr = oldWePtr.lock();
        if (shPtr) {
            to.push_back(std::move(shPtr));
        }
    }
}

vector<tmap::MapTwigPtr> tmap::MergedExp::findTwigsUsingThis()
{
    vector<MapTwigPtr> res;
    vector<MapTwigPtr> DFS_Stack;

    if (!hasSearchedLoopClosure) {
        /// 如果是第一次搜索, 从RelatedMaps出发搜索
        hasSearchedLoopClosure = true;
        TOOLFUN_extractSharedFromWeak(DFS_Stack, mRelatedMaps);
    } else {
        /// 如果不是, 从上一次的结果出发
        TOOLFUN_extractSharedFromWeak(DFS_Stack, mLastSearchResult);
        mLastSearchResult.clear();
    }

    while (!DFS_Stack.empty())
    {
        const auto & currentTwig = DFS_Stack.back();
        /// 先检查当前的这个MapTwig有没有使用了衍生自this的MergedExp, 如果是, 则不予采用
        bool thisTwigSafe = true;
        for (const auto & mergedExpUsage : currentTwig->getExpUsages()) {
            if (mergedExpUsage->isChildOf(this)) {
                thisTwigSafe = false;
                break;
            }
        }
        if (!thisTwigSafe) {
            DFS_Stack.pop_back();
            continue;
        }
        /// 重复闭环检查完毕

        MapTwigPtr expiredCurrentTwig;

        switch (currentTwig->getStatus()) {
            case MapTwigStatus::EXPIRED:
                /// 这意味着有后续的twig, 将后代插入currentTwig继续DFS

                /// currentTwig是引用, pop后会析构, 因此删除前要先复制出来
                expiredCurrentTwig = std::move(DFS_Stack.back());
                DFS_Stack.pop_back();
                for (const auto & child : expiredCurrentTwig->getChildren()) {
                    if (!child.expired()) {
                        ///这个分支没有废弃, 继续进行搜索
                        DFS_Stack.emplace_back(child.lock());
                    }
                }
                continue;
            case MapTwigStatus::MOVE2NEW:
                /// 这是一个可能的闭环, 但是闭环发生在current的后代上, 所以依旧把current放入last result
                mLastSearchResult.emplace_back(currentTwig);
                res.emplace_back(std::move(DFS_Stack.back()));
                DFS_Stack.pop_back();
                break;
            case MapTwigStatus::MOVE2OLD:
                /// 因为MOVE2OLD, current是不可能进行闭环的, 但是有可能将来会闭环, 所以下次还要上搜索
                mLastSearchResult.emplace_back(currentTwig);
                DFS_Stack.pop_back();
                break;
            }
    }

    return res;
}

MergedExpPtr MergedExp::bornOne(ExpPtr newExp, MatchResult matchResult)
{
    Exp* expBareP = newExp.get();
    MergedExpPtr res(new MergedExp(
            shared_from_this(), std::move(newExp), std::move(matchResult)));
    mNewestChild = res;
    expBareP->addMergedExpIns(res);
    return res;
}

MergedExpPtr MergedExp::singleMergedFromExp(ExpPtr fatherExp)
{
    auto theFirstStoredInExp = fatherExp->theSingleMergedExp().lock();
    if (theFirstStoredInExp) {
        return theFirstStoredInExp;
    } else {
        theFirstStoredInExp.reset(new MergedExp(std::move(fatherExp)));
        return theFirstStoredInExp;
    }
}

void MergedExp::addRelatedMapTwig(const MapTwigPtr& twigPtr)
{
    mRelatedMaps.emplace_back(twigPtr);
}

void MergedExp::reserveTwigs(size_t n)
{
    mRelatedMaps.reserve(n);
}

const MergedExpWePtr& MergedExp::theNewestChild() const
{
    return mNewestChild;
}

double MergedExp::getPossDecConf() const
{
    return mPossDecConf;
}

MergedExp::GateConflictResult MergedExp::checkGateConflict(GateID gateID)
{
    GateConflictResult res{};

    /// 先检查this的是否被占用了
    if (gateID == mRelatedExp->getEnterGate()) {
        res.conflictExp = mRelatedExp.get();
        res.enter = true;
        return res;
    }

    if (gateID == mRelatedExp->getLeaveGate()) {
        res.conflictExp = mRelatedExp.get();
        res.enter = false;
        return res;
    }

    /// 为后续的单向遍历搜索做准备
    MergedExp* father = mFather.get();
    if (father != nullptr) {
        /// father存在, 检查是否在father里面被占用
        GateID relatedFatherGate = mGatesMapping2Father[gateID];
        while (true) {
            if (relatedFatherGate == GATEID_NO_MAPPING) {
                /// this的gate在father并没有对应的gate映射(是一个father Exp没有被发现的Gate), 那一定没有冲突
                return res;
            }

            const auto & relatedExp = father->mRelatedExp;
            if (relatedFatherGate == relatedExp->getEnterGate()) {
                res.conflictExp = father->mRelatedExp.get();
                res.enter = true;
                return res;
            }

            if (relatedFatherGate == relatedExp->getLeaveGate()) {
                res.conflictExp = father->mRelatedExp.get();
                res.enter = false;
                return res;
            }

            father = father->mFather.get();
            if (father == nullptr) {
                return res;
            }
            relatedFatherGate = father->mGatesMapping2Father[relatedFatherGate];
        }
    } else {
        return res;
    }
}

void MergedExp::setRelatedTwigsNextMove2old(const ExpPtr& arrivingSimiliarExp, GateID atGate)
{
    size_t nExpired = 0;

    for (const auto& relatedTwigWe : mRelatedMaps) {
        if (!relatedTwigWe.expired()) {
            auto relatedTwig = relatedTwigWe.lock();
            relatedTwig->setTheSimilarMergedExpForNextTime(arrivingSimiliarExp, atGate);
        } else {
            nExpired++;
        }
    }
    if (nExpired > mRelatedMaps.size() / 2) {
        cleanUpExpiredRelatedTwigs();
    }
}

void MergedExp::setRelatedTwigsNextMove2new()
{
    size_t nExpired = 0;
    for (const auto & relatedTwigWe : mRelatedMaps) {
        if (!relatedTwigWe.expired()) {
            relatedTwigWe.lock()->setMove2new();
        } else {
            nExpired++;
        }
    }
    if (nExpired > mRelatedMaps.size() / 2) {
        cleanUpExpiredRelatedTwigs();
    }
}

GateID MergedExp::mapGateFromFather(GateID gateOfFather) const
{
    GateID res = mGatesMappingFromFather[gateOfFather];
    if (res < 0) {
        cerr << FILE_AND_LINE << " find reverse gate FAIL, this should not happen!" << endl;
    }
    return res;
}

GateID MergedExp::gateMapping2Father(GateID gateOfThis)
{
    return mGatesMapping2Father[gateOfThis];
}

void MergedExp::cleanUpExpiredRelatedTwigs()
{
    vector<MapTwigWePtr> cleanRes;
    cleanRes.reserve(mRelatedMaps.size() / 2);
    for (auto & twig : mRelatedMaps) {
        if (!twig.expired()) {
            cleanRes.emplace_back(std::move(twig));
        }
    }
    mRelatedMaps.swap(cleanRes);
}

const ExpPtr& MergedExp::getTheLastExp() const
{
    return mRelatedExp;
}

const MergedExpPtr& MergedExp::getFather() const
{
    return mFather;
}

void MergedExp::mapGates(vector<GateID>& gates2map) const
{
    vector<GateID> newMap;
    newMap.reserve(gates2map.size());
    auto sizeOfThisMap = mGatesMapping2Father.size();

    for (int i = 0; i < sizeOfThisMap; ++i) {
        auto newIndex = mGatesMapping2Father[i];
        if (newIndex < 0) {
            continue;
        }
        while (newIndex >= newMap.size()) {
            /// 确保下一个newIndex不会越界
            newMap.push_back(GATEID_NO_MAPPING);
        }
        newMap[newIndex] = gates2map[i];
    }
    gates2map.swap(newMap);
}

Json::Value MergedExp::toJS() const
{
    Json::Value res;
    const MergedExp* current = this;
    while (current != nullptr) {
        res["exps"].append(current->serialOfLastExp());
        current = current->mFather.get();
    }
    res["data"] = std::move(mMergedExpData->toJS());

    return res;
}

MergedExpPtr MergedExp::madeFronJS(const Jsobj& jmergedExp)
{
    return tmap::MergedExpPtr(new MergedExp(jmergedExp));
}

ExpDataPtr MergedExp::getMergedExpData() const
{
    return mMergedExpData;
}
