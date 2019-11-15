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
