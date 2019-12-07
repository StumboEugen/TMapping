//
// Created by stumbo on 2019/11/25.
//

#include "MapTwigCollection.h"

#include "MapTwig.h"

using namespace tmap;

std::vector<MapTwigPtr>& tmap::MapTwigCollection::getAliveMaps()
{
    return mAliveMaps;
}

MapTwigPtr MapTwigCollection::bornOne(const MapTwigPtr& father, double xConf)
{
    mNextGeneration.reserve(mAliveMaps.size());
    MapTwigPtr newTwig = father->bornOne(nextSerialN++, father->getConfidence() * xConf);
    mNextGeneration.push_back(newTwig);

    return newTwig;
}

void MapTwigCollection::add2NextGeneration(MapTwigPtr&& mapTwig)
{
    mNextGeneration.emplace_back(std::move(mapTwig));
}
