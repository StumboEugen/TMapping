//
// Created by stumbo on 2020/3/18.
//

#include <tuple>
#include "SubNode.h"

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
