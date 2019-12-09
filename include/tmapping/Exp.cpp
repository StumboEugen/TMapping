//
// Created by stumbo on 2019/12/2.
//

#include "Exp.h"

using namespace tmap;
using namespace std;

Exp::Exp(ExpDataPtr expData, int32_t enterGateID)
        : mData(std::move(expData)),
          mEnterGate(enterGateID),
          mMergedExps(1)
{}

const ExpDataPtr& Exp::expData() const
{
    return mData;
}

size_t Exp::serial() const
{
    return nSerial;
}

void Exp::setSerial(size_t serial)
{
    nSerial = serial;
}

void Exp::setLeftGate(int32_t leftGate)
{
    mLeftGate = leftGate;
}

const std::vector<MergedExpWePtr>& Exp::getMergedExps() const
{
    return mMergedExps;
}

void Exp::cleanUpMergedExps()
{
    auto size = mMergedExps.size();
    vector<MergedExpWePtr> newList(size);
    newList.emplace_back(std::move(mMergedExps.front()));
    for (std::size_t i = 1; i < size; ++i) {
        if (!mMergedExps[i].expired()) {
            newList.emplace_back(std::move(mMergedExps[i]));
        }
    }
    mMergedExps.swap(newList);
}

void Exp::addMergedExpIns(const MergedExpPtr& newMerged)
{
    mMergedExps.emplace_back(newMerged);
}

MergedExpWePtr& Exp::theSingleMergedExp()
{
    return mMergedExps.front();
}

int32_t Exp::getEnterGate() const
{
    return mEnterGate;
}

int32_t Exp::getLeftGate() const
{
    return mLeftGate;
}
