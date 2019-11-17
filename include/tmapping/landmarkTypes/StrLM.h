//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_STRLM_H
#define TMAPPING_STRLM_H

#include <string>

#include "Landmark.h"

namespace tmap
{

class StrLM : public Landmark
{
    std::string str;

public:
    LandmarkType type() override { return LandmarkType::StrLM; }

    explicit StrLM(std::string str = "");

    const std::string& getStr() const;
};
}


#endif //TMAPPING_STRLM_H