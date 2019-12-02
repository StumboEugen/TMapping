//
// Created by stumbo on 2019/11/17.
//

#ifndef TMAPPING_STRUCTEDMAP_H
#define TMAPPING_STRUCTEDMAP_H

#include <vector>
#include "expDataTypes/ExpDataTypes.h"
#include "gateTypes/GateTypes.h"
#include "tools/TopoVec2.h"

namespace tmap
{

class StructedMap
{
    struct TopoNode
    {
        // TODO exp的感念变化, 重构这个部分
        std::vector<ExpDataPtr> relatedExps;
        std::vector<TopoVec2> poseTransOfExps;
        /// 包含有所有有效gate的并集
        std::vector<GateUnPtr> gates;
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
