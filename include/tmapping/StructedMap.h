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

class StructedMap
{
    struct TopoNode
    {
        MergedExpPtr relatedExp;
        std::vector<TopoNode*> connections;
        ExpDataType expType;
    };

    struct AgentPos
    {
        size_t placeNode;
        TopoVec2 pos;
    };

    std::vector<TopoNode> nodes;

};

}


#endif //TMAPPING_STRUCTEDMAP_H
