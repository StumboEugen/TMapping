//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_INTERSECTION_H
#define TMAPPING_INTERSECTION_H

#include "ExpData.h"

namespace tmap
{

class Intersection : public ExpData
{

public:
    ExpType type() override { return ExpType::Intersection; }

};
}


#endif //TMAPPING_INTERSECTION_H
