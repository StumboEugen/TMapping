//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_SMALLROOM_H
#define TMAPPING_SMALLROOM_H

#include "ExpData.h"

namespace tmap
{

class SmallRoom : public ExpData
{


public:
    ExpDataType type() override { return ExpDataType::SmallRoom; }
};
}


#endif //TMAPPING_SMALLROOM_H
