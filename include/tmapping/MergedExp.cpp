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
          mGatesMapping(std::move(matchResult->gateMapping2this)),
          mPossDecConf(matchResult->possibility)
{}

MergedExp::MergedExp(ExpPtr fatherExp)
        : mFather(nullptr),
          mRelatedExp(std::move(fatherExp)),
          nMergedExps(1),
          mMergedExpData(fatherExp->expData()),
          mTrans(),
          mGatesMapping(),
          mPossDecConf(1.0)
{}

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
    to.reserve(from.capacity());
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
                /// currentTwig是引用, pop后会析构, 因此删除前要先复制出来
                expiredCurrentTwig = std::move(DFS_Stack.back());
                DFS_Stack.pop_back();
                /// 这意味着有后续的twig, 将后代插入currentTwig继续DFS
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
                break;
            case MapTwigStatus::MOVE2OLD:
                /// 因为MOVE2OLD, current是不可能进行闭环的, 但是有可能将来会闭环, 所以下次还要上搜索
                mLastSearchResult.emplace_back(currentTwig);
                break;
            }
        DFS_Stack.pop_back();
    }

    return res;
}

MergedExpPtr MergedExp::bornOne(ExpPtr newExp, MatchResult matchResult)
{
    MergedExpPtr res(new MergedExp(
            shared_from_this(), std::move(newExp), std::move(matchResult)));
    mNewestChild = res;
    return res;
}

MergedExpPtr MergedExp::bornFromExp(ExpPtr fatherExp)
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

bool MergedExp::checkIfGateIsOccupied(size_t gateID)
{
    /// 先检查this的是否被占用了
    if (gateID == mRelatedExp->getEnterGate() ||
        gateID == mRelatedExp->getLeftGate()) {
        return true;
    }
    /// 为后续的单向遍历搜索做准备
    size_t relatedFatherGate = gateID;
    MergedExpPtr& father = mFather;
    while (father != nullptr) {
        /// father存在, 检查是否在father里面被占用
        relatedFatherGate = father->mGatesMapping[relatedFatherGate];
        const auto & relatedExp = father->mRelatedExp;
        if (relatedFatherGate == relatedExp->getEnterGate() ||
            relatedFatherGate == relatedExp->getLeftGate()) {
            return true;
        }
        father = father->mFather;
    }
    return false;
}
