//
// Created by stumbo on 2019/11/15.
//

#include "GateWay.h"

Json::Value tmap::GateWay::toJS() const
{
    Json::Value res(Gate::toJS());
    res["type"] = "W";
    return res;
}
