//
// Created by stumbo on 2019/11/25.
//

#include "MapTwigCollection.h"

#include "MapTwig.h"

using namespace tmap;

std::unordered_set<MapTwigPtr>& tmap::MapTwigCollection::getAliveMaps()
{
    return aliveMaps;
}

void MapTwigCollection::killAliveMap(MapTwigPtr map2kill)
{
    map2kill->setExpired();
    aliveMaps.erase(map2kill);
}
