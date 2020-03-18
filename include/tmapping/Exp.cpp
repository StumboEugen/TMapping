//
// Created by stumbo on 2019/12/2.
//

#include "Exp.h"

using namespace tmap;
using namespace std;

Exp::Exp(ExpDataPtr expData, GateID enterGateID)
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

void Exp::setLeftGate(GateID leaveGate)
{
    mLeaveGate = leaveGate;
}

const std::vector<MergedExpWePtr>& Exp::getMergedExps() const
{
    return mMergedExps;
}

void Exp::cleanUpExpiredMergedExps()
{
    auto size = mMergedExps.size();
    vector<MergedExpWePtr> newList;
    newList.reserve(mMergedExps.size() / 2);
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

/**
 * @brief 获得Exp的第一个MergedExp, 由于构造时第一个位置是一定被空出来的,所以不会发生SEG
 */
MergedExpWePtr& Exp::theSingleMergedExp()
{
    return mMergedExps.front();
}

GateID Exp::getEnterGate() const
{
    return mEnterGate;
}

GateID Exp::getLeaveGate() const
{
    return mLeaveGate;
}

Jsobj Exp::toJS() const
{
    Jsobj res;
    res["data"] = std::move(mData->toJS());
    res["serial"] = nSerial;
    res["enterGate"] = mEnterGate;
    res["leaveGate"] = mLeaveGate;
    return res;
}

Exp::Exp(const Jsobj& jexp)
        : mData(ExpData::madeFromJS(jexp["data"]))
{
    nSerial = jexp["serial"].asUInt64();
    mEnterGate = jexp["enterGate"].asInt();
    mLeaveGate = jexp["leaveGate"].asInt();
}
