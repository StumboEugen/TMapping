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
        : mPos(pos),
          mNormalVec(normalVec.unitize())
{}

const tmap::TopoVec2& tmap::Gate::getPos() const
{
    return mPos;
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
    res["pos"] = std::move(mPos.toJS());
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
    if (type == typeStr(GateType::Door)) {
        res.reset(new Door(p, nv, jgate["opened"].asBool(), jgate["mark"].asString()));
    }
    else if (type == typeStr(GateType::GateWay)) {
        res.reset(new GateWay(p, nv));
    }
    else {
        cerr << FILE_AND_LINE << " You input an UNKNOWN gate! typeStr=" << type << endl;
        return res;
    }
    res->possibility = jgate["psb"].asDouble();
    return res;
}

std::string tmap::Gate::typeStr(tmap::GateType type)
{
    string res;
    switch (type) {
        case GateType::GateWay:
            res = "GateWay";
            break;
        case GateType::DoorClosed:
        case GateType::DoorOpened:
        case GateType::Door:
            res = "Door";
            break;
    }
    return res;
}

void tmap::Gate::copy2(tmap::Gate* target)
{
    target->mNormalVec = this->mNormalVec;
    target->mPos = this->mPos;
    target->possibility = this->possibility;
}

void tmap::Gate::setPos(const tmap::TopoVec2& pos)
{
    Gate::mPos = pos;
}

void tmap::Gate::setNormalVec(const tmap::TopoVec2& normalVec)
{
    Gate::mNormalVec = normalVec.unitize();
}

bool tmap::Gate::alike(const tmap::GatePtr& that) const
{
    ///DEBUG SFAE TODO REMOVE
    if (abs(this->mNormalVec.len() - 1.0) > 0.1 ||
        abs(that->mNormalVec.len() - 1.0) > 0.1) {
        cerr << FILE_AND_LINE << " normalVec not unit!" << endl;
    }

    double res = this->mNormalVec.dotProduct(that->mNormalVec);
    return res > 0.85;
}

void tmap::Gate::mergeBasicInfo(const Gate* A, const Gate* B,
                                const tmap::TopoVec2& BPos, double thisWeight)
{
    this->mPos = (A->mPos * thisWeight + BPos) / (thisWeight + 1);
    this->mNormalVec = (A->mNormalVec * thisWeight + B->mNormalVec) / (thisWeight + 1);
    this->possibility = (A->possibility * thisWeight + B->possibility) / (thisWeight + 1);
}
