//
// Created by stumbo on 2019/11/15.
//

#include "Corridor.h"

#include <cmath>
#include <iostream>

using namespace std;
using namespace tmap;

Json::Value tmap::Corridor::toJS() const
{
    Json::Value res(ExpData::toJS());
    res["endG_A"] = mEndGateA;
    res["endG_B"] = mEndGateB;
    res["endP_A"] = std::move(mEndPointA.toJS());
    res["endP_B"] = std::move(mEndPointB.toJS());
    return res;
}

tmap::ExpDataPtr tmap::Corridor::clone() const
{
    auto c = new Corridor;
    this->copy2(c);
    c->mEndGateA = this->mEndGateA;
    c->mEndGateB = this->mEndGateB;
    c->mEndPointA = this->mEndPointA;
    c->mEndPointB = this->mEndPointB;
    return tmap::ExpDataPtr(c);
}

ExpDataPtr Corridor::cloneShell() const
{
    auto c = new Corridor;
    c->mEndGateA = this->mEndGateA;
    c->mEndGateB = this->mEndGateB;
    c->mEndPointA = this->mEndPointA;
    c->mEndPointB = this->mEndPointB;
    return tmap::ExpDataPtr(c);
}

tmap::GateID tmap::Corridor::getEndGateA() const
{
    return mEndGateA;
}

tmap::GateID tmap::Corridor::getEndGateB() const
{
    return mEndGateB;
}

void tmap::Corridor::setEndGateA(tmap::GateID endGateA)
{
    mEndGateA = endGateA;
    if (endGateA >= 0) {
        mEndPointA = getGates()[endGateA]->getPos();
    }
}

void tmap::Corridor::setEndGateB(tmap::GateID endGateB)
{
    mEndGateB = endGateB;
    if (endGateB >= 0) {
        mEndPointB = getGates()[endGateB]->getPos();
    }
}

const tmap::TopoVec2& tmap::Corridor::getEndPointA() const
{
    return mEndPointA;
}

const tmap::TopoVec2& tmap::Corridor::getEndPointB() const
{
    return mEndPointB;
}

void tmap::Corridor::setEndPointA(const tmap::TopoVec2& endPointA)
{
    mEndPointA = endPointA;
}

void tmap::Corridor::setEndPointB(const tmap::TopoVec2& endPointB)
{
    mEndPointB = endPointB;
}

std::array<double, 4> tmap::Corridor::getOutBounding(double expandValue) const
{
    auto res = ExpData::getOutBounding(0.);
    if (mEndGateA < 0) {
        auto& pos = mEndPointA;
        res[0] = max(res[0], pos.py);
        res[1] = min(res[1], pos.py);
        res[2] = min(res[2], pos.px);
        res[3] = max(res[3], pos.px);
    }

    if (mEndGateB < 0) {
        auto& pos = mEndPointB;
        res[0] = max(res[0], pos.py);
        res[1] = min(res[1], pos.py);
        res[2] = min(res[2], pos.px);
        res[3] = max(res[3], pos.px);
    }
    res[0] += expandValue;
    res[1] -= expandValue;
    res[2] -= expandValue;
    res[3] += expandValue;
    return res;
}

void tmap::Corridor::moveGatePos(tmap::GateID id, const tmap::TopoVec2& newPos)
{
    if (id < 0 || id >= this->nGates()) {
        cerr << FILE_AND_LINE << " An invalid GateID!!! id:" << id;
        return;
    }

    if (id == mEndGateA || id == mEndGateB) {
        auto AB = mEndPointB - mEndPointA;
        auto nAB = AB.unitize();
        double lenAB = AB.len();

        auto gSize = mGates.size();
        vector<double> lenRatio(gSize, 0.0);

        for (int i = 0; i < gSize; ++i) {
            if (i == mEndGateB || i == mEndGateA) {
                lenRatio[i] = 0.0;
                continue;
            }
            auto AC = mGates[i]->getPos() - mEndPointA;
            lenRatio[i] = AC.dotProduct(nAB) / lenAB;
            if (AB.crossProduct(AC) < 0.) {
                lenRatio[i] *= -1;
            }
        }

        if (id == mEndGateA) {
            mEndPointA = newPos;
            this->getGates()[id]->setPos(newPos);
        } else {
            mEndPointB = newPos;
            this->getGates()[id]->setPos(newPos);
        }

        AB = mEndPointB - mEndPointA;
        lenAB = AB.len();
        for (int i = 0; i < gSize; ++i) {
            if (lenRatio[i] == 0.0) {
                continue;
            }
            auto AD = AB.changeLen(abs(lenRatio[i]) * lenAB);
            auto nv = AD.unitize().rotate(lenRatio[i] > 0. ? 90 : -90);
            auto C = mEndPointA + AD + nv * halfWidth();
            mGates[i]->setPos(C);
            mGates[i]->setNormalVec(nv);
        }
    }
}

tmap::TopoVec2 tmap::Corridor::normalizeSelf()
{
    auto offset = ExpData::normalizeSelf();
    mEndPointB -= offset;
    mEndPointA -= offset;
    return offset;
}

pair<TopoVec2, TopoVec2> tmap::Corridor::calPosAmdNvFromPointC(const tmap::TopoVec2& C)
{
    auto AB = this->getEndPointB() - this->getEndPointA();
    auto AC = C - this->getEndPointA();
    auto nAB = AB.unitize();
    auto projectP = nAB * nAB.dotProduct(AC) + this->getEndPointA();
    bool isLeftSide = AB.crossProduct(AC) > 0.;
    auto nv = nAB.rotate(isLeftSide ? 90 : -90);
    projectP += nv * halfWidth();
    return make_pair(projectP, nv);
}

double Corridor::halfWidth() const
{
    return 0.35;
}
