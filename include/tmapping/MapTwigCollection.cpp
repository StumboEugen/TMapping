//
// Created by stumbo on 2019/11/25.
//

#include "MapTwigCollection.h"

#include "MapTwig.h"
#include "MergedExp.h"

#include <algorithm>
#include <iostream>

using namespace tmap;
using namespace std;

std::vector<MapTwigPtr>& tmap::MapTwigCollection::getAliveMaps()
{
    return mAliveMaps;
}

MapTwigPtr MapTwigCollection::bornOne(const MapTwigPtr& father, const MergedExpPtr& newMerged)
{
    MapTwigPtr newTwig = father->bornOne(nextSerialN++);
    newTwig->addMergedExp(newMerged);
    newMerged->addRelatedMapTwig(newTwig);
    mNextGeneration.push_back(newTwig);
    mMapTwigs.emplace_back(newTwig);
    return newTwig;
}

MapTwigPtr MapTwigCollection::generateAdam()
{
    auto adam = MapTwig::getAdamTwig();
    nextSerialN = 1;
    mNextGeneration.push_back(adam);
    mMapTwigs.emplace_back(adam);
    return adam;
}

void MapTwigCollection::add2NextGeneration(MapTwigPtr&& mapTwig)
{
    mNextGeneration.emplace_back(std::move(mapTwig));
}

size_t MapTwigCollection::nextgCompleteAdding(size_t nSurviver, size_t experienceCount)
{
    if (mNextGeneration.empty()) {
        cerr << FILE_AND_LINE << "ALL map out!" << endl;
        throw;
    }

    if (nSurviver == 0 || nSurviver > mNextGeneration.size()) {
        nSurviver = mNextGeneration.size();
    }

    for (auto& twig : mNextGeneration) {
        twig->resetLastGlobalConfidenceResult();
    }

    double logN = log(experienceCount);
    auto comp = [logN](const MapTwigPtr& a, const MapTwigPtr& b) {
        return a->calGlobalPoss(logN) > b->calGlobalPoss(logN);
    };

    partial_sort(mNextGeneration.begin(),
                 mNextGeneration.begin() + nSurviver,
                 mNextGeneration.end(), comp);

    mNextGeneration.erase(mNextGeneration.begin() + nSurviver, mNextGeneration.end());

    mAliveMaps.swap(mNextGeneration);
    mNextGeneration.clear();

    mPossScores.clear();
    mPossScores.reserve(mAliveMaps.size());
    double sum = 0.0;
    for (const auto& aliveMap : mAliveMaps) {
        double poss = aliveMap->calGlobalPoss(logN);
        sum += poss;
        mPossScores.push_back(poss);
    }
    for (auto& score : mPossScores) {
        score /= sum;
    }

    return nSurviver;
}

const vector<double>& MapTwigCollection::getScores() const
{
    return mPossScores;
}
