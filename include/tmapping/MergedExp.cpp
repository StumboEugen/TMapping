//
// Created by stumbo on 2019/12/2.
//

#include "MergedExp.h"

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
