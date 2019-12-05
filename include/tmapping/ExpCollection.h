//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_EXPCOLLECTION_H
#define TMAPPING_EXPCOLLECTION_H

#include "expDataTypes/ExpDataTypes.h"
#include "tools/TopoTools.h"
#include "Exp.h"
#include "MapTwigCollection.h"

#include <memory>
#include <unordered_map>
#include <map>
#include <set>

namespace tmap
{

class ExpCollection
{
    size_t mSerial;
    std::vector<ExpPtr> mExperiencesData;
    std::map<ExpDataType, std::vector<Exp*>> mClassification;

public:
    void setLeftGateOfCurrent(size_t leftGate);
    void setLeftGateOfCurrent(const TopoVec2& gatePos);

    void addNewExpAndAddLoopClosures(tmap::ExpPtr expPtr,
                                     tmap::MapTwigCollection& twigMaster);
};

}


#endif //TMAPPING_EXPCOLLECTION_H
