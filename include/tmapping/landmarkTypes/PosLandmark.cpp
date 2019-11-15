//
// Created by stumbo on 2019/11/15.
//

#include "PosLandmark.h"

tmap::PosLandmark::PosLandmark(const tmap::TopoVec2& pos) : pos(pos)
{}

const tmap::TopoVec2& tmap::PosLandmark::getPos() const
{
    return pos;
}
