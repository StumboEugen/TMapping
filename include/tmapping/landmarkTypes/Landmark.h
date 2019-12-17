//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_LANDMARK_H
#define TMAPPING_LANDMARK_H

#include "../tools/TopoParams.h"

namespace tmap
{

enum class LandmarkType {StrLM, NumLM, StrPLM};

class Landmark
{

public:
    virtual LandmarkType type() = 0;

    virtual Json::Value toJS() const;

};
}


#endif //TMAPPING_LANDMARK_H
