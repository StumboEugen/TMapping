//
// Created by stumbo on 2019/12/2.
//

#include "MergedExp.h"
#include "Exp.h"
#include "MapTwig.h"

#include <iostream>

using namespace std;
using namespace tmap;

double tmap::MergedExp::alike(const tmap::MergedExp& another) const
{
    double weight = this->nMergedExps;
    weight /= another.nMergedExps;
    return mergedExpData->alike(*another.mergedExpData, weight);
}

double tmap::MergedExp::alike(const ExpData& expData) const
{
    return mergedExpData->alike(expData, this->nMergedExps);
}

size_t tmap::MergedExp::lastExpSerial() const
{
    return mRelatedExp->serial();
}

bool tmap::MergedExp::isLastButOneExpSerial(size_t serial2Check) const
{
    if (mFather) {
        if (mFather->mRelatedExp->serial() == serial2Check) {
            return true;
        }
    }
    return false;
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

vector<tmap::MapTwigPtr> tmap::MergedExp::getLoopClosureMaps()
{
    vector<MapTwigPtr> res;
    vector<MapTwigPtr> DFS_Stack;
    size_t thisSerial = lastExpSerial();

    /// DFS_Stack
    if (!hasSearchedLoopClosure) {
        hasSearchedLoopClosure = true;
        TOOLFUN_extractSharedFromWeak(DFS_Stack, mRelatedMaps);
    } else {
        TOOLFUN_extractSharedFromWeak(DFS_Stack, mLastSearchResult);
        mLastSearchResult.clear();
    }

    while (!DFS_Stack.empty())
    {
        const auto & currentTwig = DFS_Stack.back();
        /// 先检查当前的这个分支有没有已经产生重复闭环
        bool thisTwigSafe = true;
        for (const auto & closureIns : currentTwig->getLoopClosures()) {
            if (closureIns->isLastButOneExpSerial(thisSerial)) {
                thisTwigSafe = false;
                break;
            }
        }
        if (!thisTwigSafe) {
            DFS_Stack.pop_back();
            continue;
        }
        /// 重复闭环检查完毕

        switch (currentTwig->getStatus()) {
            case MapTwigStatus::EXPIRED:
                /// 这意味着有后续的twig, 继续DSF
                for (const auto & child : currentTwig->getChildren()) {
                    if (!child.expired()) {
                        ///这个分支没有废弃, 继续进行搜索
                        DFS_Stack.emplace_back(child.lock());
                    }
                }
                break;
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