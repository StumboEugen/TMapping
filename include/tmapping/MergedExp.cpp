//
// Created by stumbo on 2019/12/2.
//

#include "MergedExp.h"

#include <iostream>

using namespace std;

double tmap::MergedExp::alike(const tmap::MergedExp& another)
{
    double weight = this->relatedExps.size();
    weight /= another.relatedExps.size();
    return mergedExpData->alike(*another.mergedExpData, weight);
}

double tmap::MergedExp::alike(const ExpData& expData)
{
    return mergedExpData->alike(expData, this->relatedExps.size());
}


tmap::MergedExp::Fast2DGateID_::Fast2DGateID_(size_t dimX, size_t dimY) : mDimX(dimX),
                                                                          mDimY(dimY),
                                                                          data(dimX * dimY)
{}

tmap::gateID& tmap::MergedExp::Fast2DGateID_::at(size_t x, size_t y)
{
    if (x >= mDimX || y >= mDimY) {
        cerr << FILE_AND_LINE << " Out of range!" << endl;
        throw;
    }
    return data[x * mDimY + y];
}

const tmap::gateID& tmap::MergedExp::Fast2DGateID_::at(size_t x, size_t y) const
{
    if (x >= mDimX || y >= mDimY) {
        cerr << FILE_AND_LINE << " Out of range!" << endl;
        throw;
    }
    return data[x * mDimY + y];
}
