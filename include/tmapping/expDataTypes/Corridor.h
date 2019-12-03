//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_CORRIDOR_H
#define TMAPPING_CORRIDOR_H

#include "ExpData.h"

namespace tmap
{

class Corridor : public ExpData
{
    bool completed;

public:
    explicit Corridor(bool completed);

    ExpDataType type() const override { return ExpDataType::Corridor; }

};
}


#endif //TMAPPING_CORRIDOR_H
