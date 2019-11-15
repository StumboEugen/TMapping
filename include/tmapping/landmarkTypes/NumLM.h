//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_NUMLM_H
#define TMAPPING_NUMLM_H

#include <vector>

#include "Landmark.h"

namespace tmap
{

class NumLM : public Landmark
{
    std::vector<double> features;

public:
    LandmarkType type() override { return LandmarkType::NumLM; }

    void setFeatures(const std::vector<double>& features);

    const std::vector<double>& getFeatures() const;
};
}


#endif //TMAPPING_NUMLM_H
