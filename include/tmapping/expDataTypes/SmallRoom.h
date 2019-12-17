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
    ExpDataType type() const override { return ExpDataType::SmallRoom; }

    Json::Value toJS() const override;
};
}


#endif //TMAPPING_SMALLROOM_H
