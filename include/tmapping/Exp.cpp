//
// Created by stumbo on 2019/12/2.
//

#include "Exp.h"

using namespace tmap;

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
    while (!mMergedExps.empty()) {
        if (mMergedExps.back().expired()) {
            mMergedExps.pop_back();
        } else {
            break;
        }
    }
}
