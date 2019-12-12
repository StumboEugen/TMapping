//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPCOLLECTION_H
#define TMAPPING_EXPCOLLECTION_H

#include "expDataTypes/ExpDataTypes.h"
#include "tools/TopoTools.h"

#include <memory>
#include <unordered_map>
#include <map>
#include <set>

namespace tmap
{

class MapTwigCollection;

class ExpCollection
{
    std::vector<ExpPtr> mExperiencesData;
    std::map<ExpDataType, std::vector<Exp*>> mClassification;

public:
    /**
     * @brief 设置所有的mapTwig离开对应的gate, 同时推算每个MapTwig接下来的status
     * @param leftGate 离开的gate序号
     */
    void setLeftGateOfCurrent(size_t leftGate);
    void setLeftGateOfCurrent(const TopoVec2& gatePos);

    /**
     * @brief 将新的Exp添加到集合中, 并且建立和这个exp相关的可行闭环地图
     * @param newExp 新的Exp经历
     * @param twigMaster MapTwig的管理集合
     */
    void addNewExpAndAddLoopClosures(tmap::ExpPtr newExp,
                                     tmap::MapTwigCollection& twigMaster);

    const ExpPtr& getExpAt(size_t serial) const;
};

}


#endif //TMAPPING_EXPCOLLECTION_H
