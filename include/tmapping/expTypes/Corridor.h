//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_CORRIDOR_H
#define TMAPPING_CORRIDOR_H

#include "Exp.h"

namespace tmap
{

class Corridor : public Exp
{
    bool completed;

public:
    explicit Corridor(bool completed);

    ExpType type() override { return ExpType::Corridor; }

};
}


#endif //TMAPPING_CORRIDOR_H
