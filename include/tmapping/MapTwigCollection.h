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
    std::vector<MapTwigPtr> mAliveMaps;
    /// 所有MapTwig的weakPtr
    std::vector<MapTwigWePtr> mMapTwigs;
    /// 方便高速地处理下一代的问题
    std::vector<MapTwigPtr> mNextGeneration;
    /// 各个aliveMaps的对应分数
    std::vector<double> mPossScores;

public:
    std::vector<MapTwigPtr>& getAliveMaps();

    /// 生产后代Twig, 自动加入this->nextGeneration, 自动serial编号, 自动设置父子关系,
    /// 自动为MapTwig添加新的MergedExp, 自动在newMerged中注册新的MapTwig
    MapTwigPtr bornOne(const MapTwigPtr& father, const MergedExpPtr& newMerged);

    MapTwigPtr generateAdam();

    void add2NextGeneration(MapTwigPtr&& mapTwig);

    size_t nextgCompleteAdding(size_t nSurviver, size_t experienceCount);

    const std::vector<double>& getScores() const;
};

}


#endif //TMAPPING_INCLUDE_TMAPPING_MAPTWIGCOLLECTION_H
