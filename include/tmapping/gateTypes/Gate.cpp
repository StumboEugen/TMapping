//
// Created by stumbo on 2019/11/15.
//

#include "Gate.h"

tmap::Gate::Gate(const tmap::TopoVec2& pos, const tmap::TopoVec2& normalVec)
:pos(pos),
 normalVec(normalVec)
{}

const tmap::TopoVec2& tmap::Gate::getPos() const
{
    return pos;
}

const tmap::TopoVec2& tmap::Gate::getNormalVec() const
{
    return normalVec;
}

void tmap::Gate::setPossibility(double possibility)
{
    Gate::possibility = possibility;
}

double tmap::Gate::getPossibility() const
{
    return possibility;
}

Json::Value tmap::Gate::toJS() const
{
    Json::Value res;
    res["pos"] = std::move(pos.toJS());
    res["nv"] = std::move(normalVec.toJS());
    res["psb"] = possibility;
    return res;
}
