//
// Created by stumbo on 2019/11/15.
//

#include "Landmark.h"

#include <string>

using namespace std;

Json::Value tmap::Landmark::toJS() const
{
    Jsobj res;
    res["type"] = typeStr(type());
    return res;
}

std::string tmap::Landmark::typeStr(LandmarkType type)
{
    string res;
    switch (type) {
        case LandmarkType::StrLM:
            res = "S";
            break;
        case LandmarkType::NumLM:
            res = "N";
            break;
        case LandmarkType::StrPLM:
            res = "SP";
            break;
    }
    return res;
}

double tmap::Landmark::getPossibility() const
{
    return mPoss;
}

void tmap::Landmark::setPossibility(double poss)
{
    Landmark::mPoss = poss;
}
