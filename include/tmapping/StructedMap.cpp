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
    res["version"] = TOPO_JSON_VERSION;
    res["poss"] = mPossibility;
    res["agentAt"] = mAgentAt;

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

StructedMapImpl::StructedMapImpl(const Json::Value& jmap)
{
    /// 输入poss和agentPos
    mPossibility = jmap["poss"].asDouble();
    mAgentAt = jmap["agentAt"].asUInt64();

    /// 先分配nodes的空间, 因为link需要设置到对应的nodePtr上
    const auto& jnodes = jmap["nodes"];
    auto nodeSize = jnodes.size();
    mNodes.assign(nodeSize, make_shared<MapNode>());
    for (int i = 0; i < nodeSize; ++i) {
        mNodes[i]->serial = i;
    }

    for (int i = 0; i < nodeSize; ++i) {
        const auto& jnode = jnodes[i];
        const auto& jlinks = jnode["links"];
        auto jlinkSize = jlinks.size();
        auto& mapNode = mNodes[jnode["serial"].asUInt64()];
        auto& links = mapNode->links;
        links.assign(jlinkSize, {});
        /// 设置所有link
        for (int j = 0; j < jlinkSize; ++j) {
            const auto& jlink = jlinks[j];
            auto& link = links[j];
            link.to = mNodes[jlink["t"].asUInt64()];
            link.at = jlink["@"].asInt();
        }
        /// 调用static工厂函数, 生成核心数据
        mapNode->relatedMergedExp = MergedExp::madeFronJS(jnode["exp"]);
    }
}
