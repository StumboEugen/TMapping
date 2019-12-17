//
// Created by stumbo on 2019/11/17.
//

#include "StructedMap.h"

#include <utility>

tmap::StructedMapImpl::StructedMapImpl(std::vector<MapNodePtr> nodes,
                                       const MapTwigPtr& twigUsed,
                                       double poss) :
        mNodes(std::move(nodes)),
        mRelatedTwig(twigUsed),
        mAgentAt(nodes.size() - 1),
        mPossibility(poss)
{
}

const tmap::MapTwigWePtr& tmap::StructedMapImpl::relatedTwig() const
{
    return mRelatedTwig;
}
