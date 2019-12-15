//
// Created by stumbo on 2019/11/17.
//

#ifndef TMAPPING_STRUCTEDMAP_H
#define TMAPPING_STRUCTEDMAP_H

#include <vector>

#include "tools/TopoParams.h"
#include "tools/TopoVec2.h"
#include "expDataTypes/ExpDataTypes.h"

namespace tmap
{

struct MapNode
{
    struct Link {
        MapNodeWe to;
        size_t at;
    };
    MergedExpPtr relatedMergedExp;
    std::vector<Link> links;
};

class StructedMapImpl
{
//    struct AgentPos
//    {
//        size_t placeNode;
//        TopoVec2 pos;
//    };

    std::vector<MapNodePtr> mNodes;

    MapTwigWePtr mRelatedTwig;

    size_t mAgentAt;

public:
    StructedMapImpl(std::vector<MapNodePtr> nodes, const MapTwigPtr& twigUsed);

    const MapTwigWePtr& relatedTwig() const;
};

}


#endif //TMAPPING_STRUCTEDMAP_H
