//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_SMALLROOM_H
#define TMAPPING_SMALLROOM_H

#include "Exp.h"

namespace tmap
{

class SmallRoom : public Exp
{


public:
    ExpType type() override { return ExpType::SmallRoom; }
};
}


#endif //TMAPPING_SMALLROOM_H
