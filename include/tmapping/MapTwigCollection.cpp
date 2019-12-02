//
// Created by stumbo on 2019/11/25.
//

#include "MapTwigCollection.h"

const std::unordered_map<size_t, tmap::MapTwigPtr>& tmap::MapTwigCollection::getAliveMaps() const
{
    return aliveMaps;
}
