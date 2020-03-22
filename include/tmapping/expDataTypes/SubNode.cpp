//
// Created by stumbo on 2020/3/18.
//

#include <tuple>
#include <iostream>

#include "SubNode.h"
#include "../tools/TopoTools.h"

using namespace std;

tmap::SubNode::SubNode(tmap::SubNodeType type, uint32_t index) : type(type), index(index)
{}

bool tmap::SubNode::operator==(const tmap::SubNode& rhs) const
{
    return std::tie(type, index) == std::tie(rhs.type, rhs.index);
}

bool tmap::SubNode::operator!=(const tmap::SubNode& rhs) const
{
    return !(rhs == *this);
}

uint32_t tmap::SubNode::toUIndex(uint32_t nGate) const
{
    switch (type) {
        case SubNodeType::GATE:
            return index;
        case SubNodeType::LandMark:
            return index + nGate;
        default:
            cerr << FILE_AND_LINE << " DEFAULT?!";
            return INT32_MAX;
    }
}
