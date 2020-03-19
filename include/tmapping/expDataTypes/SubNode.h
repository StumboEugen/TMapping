//
// Created by stumbo on 2020/3/18.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_SUBNODE_H
#define TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_SUBNODE_H

#include <cstdint>

namespace tmap
{

enum class SubNodeType : int32_t {
    GATE = 0,
    LandMark = 1,
    UNSET = -1
};

/// 表示ExpData中的SubNode, 可以用来表示连接关系也可以用来表示机器人位置
struct SubNode
{
    SubNodeType type;
    uint32_t index;

public:
    SubNode(SubNodeType type, uint32_t index);

    bool operator==(const SubNode& rhs) const;

    bool operator!=(const SubNode& rhs) const;

    int toInt() const;
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_EXPDATATYPES_SUBNODE_H
