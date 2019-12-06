//
// Created by stumbo on 2019/12/2.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_EXP_H
#define TMAPPING_INCLUDE_TMAPPING_EXP_H

#include <vector>

#include "expDataTypes/ExpDataTypes.h"
#include "tools/BitsHash.h"
#include "tools/TopoParams.h"

namespace tmap
{

class Exp
{
    ExpDataPtr mData;
    /// 进入时对应的Gate, -1表示这个为出发点
    int32_t mEnterGate = -1;
    /// 离开时对应的Gate, -1表示还没离开
    int32_t mLeftGate = -1;
    /// 时间上的排序序列
    std::size_t nSerial = 0;
    std::size_t nBuiltTwig;
    std::vector<MergedExpWePtr> mMergedExps;

public:
    const ExpDataPtr& expData() const;

    size_t serial() const;

    void setSerial(size_t serial);

    void setLeftGate(int32_t leftGate);

    const std::vector<MergedExpWePtr>& getMergedExps() const;

    void cleanUpMergedExps();
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_EXP_H
