//
// Created by stumbo on 2019/11/17.
//

#include "StructedMap.h"
#include "MergedExp.h"

#include <utility>

using namespace tmap;
using namespace std;

tmap::StructedMapImpl::StructedMapImpl(std::vector<MapNodePtr> nodes,
                                       const MapTwigPtr& twigUsed,
                                       double poss) :
        mNodes(std::move(nodes)),
        mRelatedTwig(twigUsed),
        mAgentAt(nodes.size() - 1),
        mPossibility(poss)
{
    for (size_t i = 0; i < mNodes.size(); ++i) {
        mNodes[i]->serial = i;
    }
}

const tmap::MapTwigWePtr& tmap::StructedMapImpl::relatedTwig() const
{
    return mRelatedTwig;
}

Json::Value tmap::StructedMapImpl::toJS() const
{
    Json::Value res;
    res["version"] = TOPO_VERSION;
    res["poss"] = mPossibility;

    for (const auto& node : mNodes) {
        Json::Value jnode;
        jnode["exp"] = node->relatedMergedExp->toJS();
        jnode["serial"] = node->serial;
        for (const auto& link : node->links) {
            Json::Value jlink;
            jlink["t"] = link.to.lock()->serial;
            jlink["@"] = link.at;
            jnode["links"].append(std::move(jlink));
        }
        res["nodes"].append(std::move(jnode));
    }
    return res;
}
