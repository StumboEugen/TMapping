//
// Created by stumbo on 2019/11/15.
//

#include "Door.h"

tmap::Door::Door(const tmap::TopoVec2& pos, const tmap::TopoVec2& normalVec, bool oepned)
        : Gate(pos, normalVec), oepned(oepned)
{}

bool tmap::Door::isOepned() const
{
    return oepned;
}
