//
// Created by stumbo on 2019/11/15.
//

#include "Gate.h"
#include "Door.h"
#include "GateWay.h"

#include <string>
#include <iostream>

using namespace std;

tmap::Gate::Gate(const tmap::TopoVec2& pos, const tmap::TopoVec2& normalVec)
        : pos(pos),
          mNormalVec(normalVec.unitize())
{}

const tmap::TopoVec2& tmap::Gate::getPos() const
{
    return pos;
}

const tmap::TopoVec2& tmap::Gate::getNormalVec() const
{
    return mNormalVec;
}

void tmap::Gate::setPossibility(double possibility)
{
    Gate::possibility = possibility;
}

double tmap::Gate::getPossibility() const
{
    return possibility;
}

Json::Value tmap::Gate::toJS() const
{
    Json::Value res;
    res["pos"] = std::move(pos.toJS());
    res["nv"] = std::move(mNormalVec.toJS());
    res["psb"] = possibility;
    res["type"] = typeStr(type());
    return res;
}

tmap::GatePtr tmap::Gate::madeFromJS(const tmap::Jsobj& jgate)
{
    tmap::GatePtr res;
    TopoVec2 p(jgate["pos"]);
    TopoVec2 nv(jgate["nv"]);
    string type = jgate["type"].asString();
    if (type == "D") {
        res.reset(new Door(p, nv, jgate["opened"].asBool(), jgate["mark"].asString()));
    }
    else if (type == "W") {
        res.reset(new GateWay(p, nv));
    }
    else {
        cerr << FILE_AND_LINE << " You input an UNKNOWN gate! type=" << type << endl;
        throw;
    }
    res->possibility = jgate["psb"].asDouble();
    return res;
}

void tmap::Gate::changeNormalVec2(const tmap::TopoVec2& to)
{
    mNormalVec = to.unitize();
}

std::string tmap::Gate::typeStr(tmap::GateType type)
{
    string res;
    switch (type) {
        case GateType::GateWay:
            res = "W";
            break;
        case GateType::DoorClosed:
        case GateType::DoorOpened:
        case GateType::Door:
            res = "D";
            break;
    }
    return res;
}
