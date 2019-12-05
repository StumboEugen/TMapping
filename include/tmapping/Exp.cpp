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
