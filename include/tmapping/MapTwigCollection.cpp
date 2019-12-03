//
// Created by stumbo on 2019/11/25.
//

#include "MapTwigCollection.h"

using namespace tmap;

std::unordered_set<MapTwigPtr>& tmap::MapTwigCollection::getAliveMaps()
{
    return aliveMaps;
}
