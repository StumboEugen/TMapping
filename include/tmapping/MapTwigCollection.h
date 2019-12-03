//
// Created by stumbo on 2019/11/25.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_MAPTWIGCOLLECTION_H
#define TMAPPING_INCLUDE_TMAPPING_MAPTWIGCOLLECTION_H

#include <unordered_set>
#include <vector>
#include "tools/TopoParams.h"

namespace tmap
{

class MapTwigCollection
{
    /// 下一个新的MapTwig的Serial ID
    size_t nextSerialN = 0;
    /// 指向当前依旧存活的地图分支的最新(尾端)MapTwig
    std::unordered_set<MapTwigPtr> aliveMaps;
    /// 所有MapTwig的weakPtr
    std::vector<MapTwigWePtr> MapTwigs;

public:
    std::unordered_set<MapTwigPtr>& getAliveMaps();


};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MAPTWIGCOLLECTION_H
