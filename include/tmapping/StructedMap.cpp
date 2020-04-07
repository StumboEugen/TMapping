//
// Created by stumbo on 2019/11/17.
//

#include "StructedMap.h"
#include "MergedExp.h"
#include "MapTwig.h"

#include <utility>
#include <iostream>

using namespace tmap;
using namespace std;

tmap::StructedMapImpl::StructedMapImpl(std::vector<MapNodePtr> nodes,
                                       const MapTwigPtr& twigUsed,
                                       double poss) :
        mNodes(std::move(nodes)),
        mRelatedTwig(twigUsed),
        mAgentAt(nodes.size() - 1),
        mPossibility(poss)
#ifdef TMAPPING_CONFIG_RECORD_POSS
       ,mPossHistory(twigUsed->getPossHistory())
#endif
{
    for (size_t i = 0; i < mNodes.size(); ++i) {
        mNodes[i]->setSerial(i);
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
        jnode["exp"] = node->getRelatedMergedExp()->toJS();
        jnode["serial"] = node->getSerial();
        for (int i = 0; i < node->nLinks(); ++i) {
            auto& link = node->linkAt(i);
            Json::Value jlink;
            if (!link.to.expired()) {
                jlink["t"] = link.to.lock()->getSerial();
            }
            jlink["@"] = link.at;
            jnode["links"].append(std::move(jlink));
        }
        res["nodes"].append(std::move(jnode));
    }

#ifdef TMAPPING_CONFIG_RECORD_POSS
    if (auto twig = this->mRelatedTwig.lock()) {
        for (const auto& poss : mPossHistory) {
            res["possHistory"].append(poss);
        }
    }
#endif
    return res;
}

StructedMapImpl::StructedMapImpl(const Json::Value& jmap)
{
    /// 输入poss和agentPos
    mPossibility = jmap["poss"].asDouble();
    mAgentAt = jmap["agentAt"].asUInt64();

    /// 先生成MapNode的空间, 因为link需要设置到对应的nodePtr上
    const auto& jnodes = jmap["nodes"];
    auto nodeSize = jnodes.size();
    mNodes.assign(nodeSize, nullptr);
    for (int i = 0; i < nodeSize; ++i) {
        const auto& jnode = jnodes[i];
        auto serial = jnode["serial"].asUInt64();
        mNodes[serial] = MapNode::makeOneFromMergedExp
                (MergedExp::madeFronJS(jnode["exp"]), serial);
    }

    for (int i = 0; i < nodeSize; ++i) {
        const auto& jnode = jnodes[i];
        const auto& jlinks = jnode["links"];
        auto jlinkSize = jlinks.size();
        auto& mapNode = mNodes[jnode["serial"].asUInt64()];
        /// 设置所有link
        for (int j = 0; j < jlinkSize; ++j) {
            const auto& jlink = jlinks[j];
            auto& link = mapNode->linkAt(j);
            if (!jlink["t"].isNull()) {
                link.to = mNodes[jlink["t"].asUInt64()];
            }
            link.at = jlink["@"].asInt();
        }
    }

#ifdef TMAPPING_CONFIG_RECORD_POSS
    mPossHistory.reserve(jmap["possHistory"].size());
    for (const auto& jPoss : jmap["possHistory"]) {
        mPossHistory.push_back(jPoss.asDouble());
    }
#endif
}

const vector<MapNodePtr>& StructedMapImpl::getNodes() const
{
    return mNodes;
}

double StructedMapImpl::getPsblt() const
{
    return mPossibility;
}

void StructedMapImpl::setPsblt(double psbly)
{
    StructedMapImpl::mPossibility = psbly;
}

///////////////////////// NEXT IS ABOUT MAPNODE //////////////////////////

size_t MapNode::getSerial() const
{
    return mSerial;
}

void MapNode::setSerial(size_t serial)
{
    MapNode::mSerial = serial;
}

void MapNode::addNewLink(const MapNodePtr& to, GateID at)
{
    mLinks.emplace_back(Link{to, at});
    setLinkAtIndex(mLinks.size() - 1, to, at);
}

void MapNode::addNewDanglingLink()
{
    addNewLink(nullptr, GATEID_NO_MAPPING);
}

void MapNode::setLinkAtIndex(size_t index, const MapNodePtr& to, GateID at)
{
    auto& link = linkAt(index);
    link.at = at;
    link.to = to;
    if (to) {
        auto& thatLink = to->linkAt(at);
        thatLink.to = shared_from_this();
        thatLink.at = index;
    }
}

MapNode::Link& MapNode::linkAt(size_t index)
{
    if (mLinks.size() <= index) {
        cout << FILE_AND_LINE <<
             " Warning! you are accessing a link that out of range!" << endl;
        while (mLinks.size() <= index) {
            mLinks.emplace_back();
        }
    }
    return mLinks[index];
}

size_t MapNode::nLinks() const
{
    return mLinks.size();
}

MapNodePtr MapNode::makeOneFromMergedExp(MergedExpPtr relatedExp, size_t serial)
{
    return tmap::MapNodePtr(new MapNode(std::move(relatedExp), serial));
}

MapNode::MapNode(MergedExpPtr relatedExp, size_t serial)
        : mRelatedMergedExp(std::move(relatedExp)),
          mSerial(serial)
{
    mLinks.assign(expData()->nGates(), {});
}

ExpDataPtr MapNode::expData() const
{
    if (mRelatedMergedExp) {
        return mRelatedMergedExp->getMergedExpData();
    } else {
        cerr << FILE_AND_LINE << " you require a expData from a MapNode that doesn't have a "
                                 "valid relatedMergedExp! " << endl;
        return nullptr;
    }
}

const MergedExpPtr& MapNode::getRelatedMergedExp() const
{
    return mRelatedMergedExp;
}

GateID MapNode::linkedGIDAt(size_t index)
{
    return linkAt(index).at;
}

MapNode::~MapNode() = default;
