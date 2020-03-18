//
// Created by stumbo on 2020/3/18.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_GEOHASH_H
#define TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_GEOHASH_H

#include "SubNode.h"
#include "../tools/TopoVec2.h"

#include <unordered_map>
#include <vector>

namespace tmap
{

class ExpData;

struct Entrance {
    SubNode base;
    SubNode node;
};

class GeoHash
{
    std::unordered_map<int64_t, std::vector<Entrance>> mTable;

    explicit GeoHash(const ExpData&, double odomErr);

    std::vector<Entrance>* hitThePos(const TopoVec2& pos);


};

}


#endif //TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_GEOHASH_H
