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
    MapTwigPtr newTwig = father->bornOne(nextSerialN++, father->getConfidence() * xConf);
    mNextGeneration.push_back(newTwig);
    mMapTwigs.emplace_back(newTwig);
    return newTwig;
}

MapTwigPtr MapTwigCollection::generateAdam()
{
    auto adam = MapTwig::getAdamTwig();
    mNextGeneration.push_back(adam);
    mMapTwigs.emplace_back(adam);
    return adam;
}

void MapTwigCollection::add2NextGeneration(MapTwigPtr&& mapTwig)
{
    mNextGeneration.emplace_back(std::move(mapTwig));
}
