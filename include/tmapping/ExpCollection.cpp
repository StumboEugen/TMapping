//
// Created by stumbo on 2019/11/15.
//

#include "ExpCollection.h"
#include "Exp.h"
#include "MergedExp.h"

void tmap::ExpCollection::setLeftGateOfCurrent(size_t leftGate)
{
    mExperiencesData.back()->mLeftGate = leftGate;
}

void tmap::ExpCollection::setLeftGateOfCurrent(const TopoVec2& gatePos)
{
    auto cloestGate = mExperiencesData.back()->mData->findTheCloestGate(gatePos);
    setLeftGateOfCurrent(cloestGate);
}

void tmap::ExpCollection::addNewExpAndFindLoopClosures(tmap::ExpPtr expPtr)
{
    auto & vecSameType = mClassification[expPtr->expData()->type()];

    for (size_t i = vecSameType.size() - 1; i >= 0; ++i) {
        auto sameTypeExp = vecSameType[i];
        double poss1 = sameTypeExp->expData()->alike(*expPtr->expData(), 1);
        if (poss1 < TOLLERANCE_1ST_MATCH_EXP) {
            continue;
        }

        //TODO 不要忘了单独闭环的情况
        auto & mergedExpCollection = sameTypeExp->mMergedExps;
        for (auto iter = mergedExpCollection.begin(); iter != mergedExpCollection.end();) {
            auto mergedExp = iter->second.lock();
            if (mergedExp) {
                double poss2 = mergedExp->alike(*expPtr->expData());
                if (poss2 > TOLLERANCE_2ND_MATCH_MERGEDEXP) {
                }
                ++iter;
            } else {
                iter = mergedExpCollection.erase(iter);
            }
        }
    }

    expPtr->nSerial = mExperiencesData.size();
    mExperiencesData.push_back(std::move(expPtr));
}
