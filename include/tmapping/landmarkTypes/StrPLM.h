//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_STRPLM_H
#define TMAPPING_STRPLM_H

#include <string>
#include "PosLandmark.h"

namespace tmap
{

class StrPLM : public PosLandmark
{
    std::string str;

public:
    LandmarkType type() const override { return LandmarkType::StrPLM; }

    StrPLM(const TopoVec2& pos, std::string str);

    const std::string& getStr() const;

    Json::Value toJS() const override;

    PLMPtr clone() override;

    bool alike(const LMPtr& that) const override;
};
}


#endif //TMAPPING_STRPLM_H
