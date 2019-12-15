//
// Created by stumbo on 2019/11/17.
//

#include "StructedMap.h"

#include <utility>

tmap::StructedMapImpl::StructedMapImpl(std::vector<MapNodePtr> nodes,
                                       const MapTwigPtr& twigUsed) :
        mNodes(std::move(nodes)),
        mRelatedTwig(twigUsed),
        mAgentAt(nodes.size() - 1)
{
}

const tmap::MapTwigWePtr& tmap::StructedMapImpl::relatedTwig() const
{
    return mRelatedTwig;
}
