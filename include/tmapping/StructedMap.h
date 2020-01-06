//
// Created by stumbo on 2019/11/17.
//

#ifndef TMAPPING_STRUCTEDMAP_H
#define TMAPPING_STRUCTEDMAP_H

#include <utility>
#include <vector>

#include "tools/TopoParams.h"
#include "tools/TopoVec2.h"
#include "expDataTypes/ExpDataTypes.h"

namespace tmap
{

struct MapNode : std::enable_shared_from_this<MapNode>
{

public:
    struct Link {
        Link() : to(), at(GATEID_NO_MAPPING) {}

        Link(const MapNodePtr& to, GateID at) : to(to), at(at) {}

        MapNodeWe to;
        GateID at = GATEID_NO_MAPPING;
    };

private:
    /// 用于方便各种构造的时候知道links[i].to指向的是哪里
    size_t mSerial = 0;

    MergedExpPtr mRelatedMergedExp;

    std::vector<Link> mLinks;

protected:
    explicit MapNode(MergedExpPtr relatedExp, size_t serial);

public:
    static MapNodePtr makeOneFromMergedExp(MergedExpPtr relatedExp, size_t serial = 0);

    void addNewLink(const MapNodePtr& to, GateID at);
    void addNewDanglingLink();

    void setLinkAtIndex(size_t index, const MapNodePtr& to, GateID at);

    Link& linkAt(size_t index);

    size_t nLinks() const;

    size_t getSerial() const;

    ExpDataPtr expData() const;

    void setSerial(size_t serial);

    const MergedExpPtr& getRelatedMergedExp() const;

    virtual ~MapNode();
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
