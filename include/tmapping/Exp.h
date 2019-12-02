//
// Created by stumbo on 2019/12/2.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_EXP_H
#define TMAPPING_INCLUDE_TMAPPING_EXP_H

#include <unordered_map>

#include "expDataTypes/ExpDataTypes.h"
#include "tools/BitsHash.h"
#include "tools/TopoParams.h"

namespace tmap
{

class Exp
{
    ExpDataPtr data;
    /// 进入时对应的Gate, -1表示这个为出发点
    int32_t enterGate = -1;
    /// 离开时对应的Gate, -1表示还没离开
    int32_t leftGate = -1;
    /// 时间上的排序序列
    std::size_t nSerial = 0;
    std::size_t nBuiltTwig;
    std::unordered_map<BitsHash, MergedExpWePtr> mergedExps;
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_EXP_H
