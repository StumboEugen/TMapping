//
// Created by stumbo on 2019/11/15.
//

#include "Intersection.h"

Json::Value tmap::Intersection::toJS() const
{
    Json::Value res(ExpData::toJS());
    return res;
}
