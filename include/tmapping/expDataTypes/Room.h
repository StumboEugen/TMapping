//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_SMALLROOM_H
#define TMAPPING_SMALLROOM_H

#include "ExpData.h"

namespace tmap
{

class Room : public ExpData
{


public:
    ExpDataType type() const override { return ExpDataType::Room; }

    Json::Value toJS() const override;

    ExpDataPtr clone() override;
};
}


#endif //TMAPPING_SMALLROOM_H
