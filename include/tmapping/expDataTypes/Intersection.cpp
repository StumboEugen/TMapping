//
// Created by stumbo on 2019/11/15.
//

#include "Intersection.h"

Json::Value tmap::Intersection::toJS() const
{
    Json::Value res(ExpData::toJS());
    return res;
}

tmap::ExpDataPtr tmap::Intersection::clone() const
{
    auto i = new Intersection;
    this->copy2(i);
    return tmap::ExpDataPtr(i);
}

tmap::ExpDataPtr tmap::Intersection::cloneShell() const
{
    auto i = new Intersection;
    return tmap::ExpDataPtr(i);
}
