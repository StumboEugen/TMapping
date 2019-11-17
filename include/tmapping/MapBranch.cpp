//
// Created by stumbo on 2019/11/17.
//

#include "MapBranch.h"

using namespace std;

tmap::MapBranchPtr tmap::MapBranch::getTheFirstOne()
{
//    MapBranchPtr newBranch{new MapBranch()};
    MapBranchPtr newBranch{new MapBranch()};
    return newBranch;
}

tmap::MapBranchPtr tmap::MapBranch::born(std::size_t atExp, double xConf)
{
//    MapBranchPtr newBranch{new MapBranch()};
    MapBranchPtr newBranch{new MapBranch()};
    newBranch->father = shared_from_this();
    newBranch->bornedAt = atExp;
    newBranch->confidence = confidence * xConf;
    children.emplace_back(newBranch);
    return newBranch;
}
