//
// Created by stumbo on 2019/12/2.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_EXP_H
#define TMAPPING_INCLUDE_TMAPPING_EXP_H

#include <vector>

#include "expDataTypes/ExpDataTypes.h"
#include "tools/TopoParams.h"

namespace tmap
{

class Exp
{
    /// 对应实际的测量数据
    const ExpDataPtr mData;
    /// 时间上的排序序列
    std::size_t nSerial = 0;
    /// 到此Exp创建过的MapTwig数量
    std::size_t nBuiltTwig = 0;
    /// 记录着包含这个Exp的所有融合假设(不包括将来的), 第一个为single mergedExp
    std::vector<MergedExpWePtr> mMergedExps;
    /// 进入时对应的Gate, GATEID_BEGINNING_POINT(-2)表示这个为出发点
    GateID mEnterGate = GATEID_BEGINNING_POINT;
    /// 离开时对应的Gate, GATEID_HAVENT_LEFT(-3)表示还没离开
    GateID mLeaveGate = GATEID_HAVENT_LEFT;

public:
    Exp(ExpDataPtr  expData, GateID enterGateID);

    MergedExpWePtr& theSingleMergedExp();

    const ExpDataPtr& expData() const;

    size_t serial() const;

    void setSerial(size_t serial);

    void setLeftGate(GateID leaveGate);

    const std::vector<MergedExpWePtr>& getMergedExps() const;

    /**
     * @brief 清理没有用的MergedExp, 对于Exp而言这是一个比较容易发生的情况
     */
    void cleanUpExpiredMergedExps();

    void addMergedExpIns(const MergedExpPtr& newMerged);

    GateID getEnterGate() const;

    GateID getLeaveGate() const;
};
}


#endif //TMAPPING_INCLUDE_TMAPPING_EXP_H
