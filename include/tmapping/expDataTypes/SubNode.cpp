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

int tmap::SubNode::toInt() const
{
    switch (type) {

        case SubNodeType::GATE:
            return index;
        case SubNodeType::LandMark:
            return -(index + 1);
        default:
            cerr << FILE_AND_LINE << " DEFAULT?!";
            return INT32_MAX;
    }
}
