//
// Created by stumbo on 2019/11/15.
//

#include "PosLandmark.h"
#include "StrPLM.h"
#include <string>
#include <iostream>

using namespace std;

tmap::PosLandmark::PosLandmark(const tmap::TopoVec2& pos) : pos(pos)
{}

const tmap::TopoVec2& tmap::PosLandmark::getPos() const
{
    return pos;
}

Json::Value tmap::PosLandmark::toJS() const
{
    Json::Value res(Landmark::toJS());
    res["pos"] = pos.toJS();
    return res;
}

tmap::PLMPtr tmap::PosLandmark::madeFromJS(const tmap::Jsobj& jPLM)
{
    PLMPtr res;
    TopoVec2 p(jPLM["pos"]);

    string type = jPLM["type"].asString();
    if (type == typeStr(LandmarkType::StrPLM)) {
        res.reset(new StrPLM(p, jPLM["str"].asString()));
    } else {
        cerr << FILE_AND_LINE << " You input an UNKNOWN PosLandmark! type=" << type << endl;
        throw;
    }
    return res;
}

void tmap::PosLandmark::setPos(const tmap::TopoVec2& pos)
{
    PosLandmark::pos = pos;
}
