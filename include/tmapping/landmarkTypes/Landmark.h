//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_LANDMARK_H
#define TMAPPING_LANDMARK_H

#include "../tools/TopoParams.h"

namespace tmap
{
class Landmark;

using LMPtr = std::shared_ptr<Landmark>;
using LMUnPtr = std::unique_ptr<Landmark>;
using LMWePtr = std::weak_ptr<Landmark>;

enum class LandmarkType {StrLM, NumLM, StrPLM};

class Landmark
{

public:
    static LMUnPtr madeFromJS(const Jsobj& jMark) = delete;

    virtual LandmarkType type() const = 0;

    virtual Json::Value toJS() const;

    static std::string typeStr(LandmarkType type);

};
}


#endif //TMAPPING_LANDMARK_H
