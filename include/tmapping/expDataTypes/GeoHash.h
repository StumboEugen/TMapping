//
// Created by stumbo on 2020/3/18.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_GEOHASH_H
#define TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_GEOHASH_H

#include "SubNode.h"
#include "../tools/TopoVec2.h"
#include "../tools/TopoParams.h"

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

public:
    explicit GeoHash(const ExpData&, double odomErr = stdErrPerMeter);

    /**
     * @brief 得到哈希表上对应位置的入口
     * @return 对应集合的指针,如果对应位置没有入口,则返回 nullptr
     */
    const std::vector<Entrance>* lookUpEntersAtPos(const TopoVec2& pos) const;

private:

    void fillEntrances(TopoVec2 midPos, double err, const SubNode& base, const SubNode& anotherNode);
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_GEOHASH_H
