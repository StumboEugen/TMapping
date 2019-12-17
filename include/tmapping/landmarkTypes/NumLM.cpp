//
// Created by stumbo on 2019/11/15.
//

#include "NumLM.h"

const std::vector<double>& tmap::NumLM::getFeatures() const
{
    return features;
}

void tmap::NumLM::setFeatures(const std::vector<double>& features)
{
    NumLM::features = features;
}

Json::Value tmap::NumLM::toJS() const
{
    Json::Value res(Landmark::toJS());
    res["type"] = "NS";
    auto& fs = res["fs"];
    for (const auto& f : features) {
        fs.append(f);
    }
    return res;
}
