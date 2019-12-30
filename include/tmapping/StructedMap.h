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
        GateID at = GATEID_NO_MAPPING;
    };

    /// 用于方便各种构造的时候知道links[i].to指向的是哪里
    size_t serial;
    MergedExpPtr relatedMergedExp;
    std::vector<Link> links;

    virtual ~MapNode() = default;
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

    double mPossibility;

public:
    StructedMapImpl(std::vector<MapNodePtr> nodes, const MapTwigPtr& twigUsed, double poss);

    explicit StructedMapImpl(const Json::Value& jmap);

    const MapTwigWePtr& relatedTwig() const;

    Json::Value toJS() const;

    const std::vector<MapNodePtr>& getNodes() const;
};

}


#endif //TMAPPING_STRUCTEDMAP_H
