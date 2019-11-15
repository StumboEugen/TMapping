//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_LANDMARK_H
#define TMAPPING_LANDMARK_H

namespace tmap
{

enum class LandmarkType {StrLM, NumLM, StrPLM};

class Landmark
{

public:
    virtual LandmarkType type() = 0;

};
}


#endif //TMAPPING_LANDMARK_H
