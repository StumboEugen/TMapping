//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_INTERSECTION_H
#define TMAPPING_INTERSECTION_H

#include "Exp.h"

namespace tmap
{

class Intersection : public Exp
{

public:
    ExpType type() override { return ExpType::Intersection; }

};
}


#endif //TMAPPING_INTERSECTION_H
