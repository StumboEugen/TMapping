//
// Created by stumbo on 2019/11/15.
//

#include <iomanip>
#include <cmath>
#include <iostream>
#include <cfloat>
#include "TopoVec2.h"
#include "TopoTools.h"

using namespace tmap;
using namespace std;

TopoVec2::TopoVec2(double px, double py) : px(px), py(py)
{}

TopoVec2& TopoVec2::operator*=(double d)
{
    px *= d;
    py *= d;
    return *this;
}

TopoVec2 TopoVec2::operator*(double d) const
{
    TopoVec2 vec = *this;
    vec *= d;
    return vec;
}

TopoVec2& TopoVec2::operator/=(double d)
{
    if (d != 0.0) {
        px /= d;
        py /= d;
    } else {
        cerr << FILE_AND_LINE << " You are trying to let a TopoVec2 / 0.0!";
    }
    return *this;
}

TopoVec2 TopoVec2::operator/(double d) const
{
    TopoVec2 vec = *this;
    vec /= d;
    return vec;
}

TopoVec2& TopoVec2::operator+=(const TopoVec2& that)
{
    px += that.px;
    py += that.py;
    return *this;
}

TopoVec2 TopoVec2::operator+(const TopoVec2& that) const
{
    TopoVec2 vec2 = that;
    vec2 += *this;
    return vec2;
}

TopoVec2& TopoVec2::operator-=(const TopoVec2& that)
{
    px -= that.px;
    py -= that.py;
    return *this;
}

TopoVec2 TopoVec2::operator-(const TopoVec2& that) const
{
    TopoVec2 vec2 = *this;
    vec2 -= that;
    return vec2;
}

double TopoVec2::len() const
{
    return sqrt(px * px + py * py);
}

ostream& tmap::operator<<(ostream& os, const TopoVec2& pos)
{
    auto oldf = os.setf(ios::fixed);
    os << '(' << pos.px << ", " << pos.py << ')';
    os.setf(oldf);
    return os;
}

Json::Value TopoVec2::toJS() const
{
    Json::Value res;
    res.append(px);
    res.append(py);
    return res;
}

TopoVec2::TopoVec2(const Jsobj& p)
        : px(p[0].asDouble()),
          py(p[1].asDouble())
{}

TopoVec2 TopoVec2::unitize() const
{
    TopoVec2 res(*this);
    double l = len();
    if (l != 0) {
        res /= l;
    } else {
        cerr << FILE_AND_LINE << " You are trying to get a unit vec from TopoVec2(0,0)!" <<
        endl;
        res.px = 1;
    }
    return res;
}

/// for convenient, not fastest
const TopoVec2& TopoVec2::restrictDir(size_t nSlice)
{
    const double dir = atan2(py, px);
    const double dStep = M_PI * 2.0 / nSlice;
    double diffMin = 10.0;
    double D = -M_PI;
    while (D <= M_PI) {
        double diffCurrent = abs(D - dir);
        if (diffCurrent < diffMin) {
            diffMin = diffCurrent;
        } else {
            break;
        }
        D += dStep;
    }
    D -= dStep;
    px = cos(D);
    py = sin(D);
    return *this;
}

TopoVec2 TopoVec2::rotate(int degree) const
{
    double rotation = M_PI / 180.0 * degree;
    const double dir = atan2(py, px);
    double d = dir + rotation;
    double l = len();
    TopoVec2 res{};
    res.px = l * cos(d);
    res.py = l * sin(d);
    return res;
}

TopoVec2 TopoVec2::changeLen(double l) const
{
    double le = this->len();
    if (le == 0.0) {
        return *this;
    }
    return *this * (l / le);
}

TopoVec2 TopoVec2::operator-() const
{
    return this->operator*(-1.0);
}

TopoVec2 TopoVec2::round2(double lidu) const
{
    if (lidu == 0.) {
        lidu = 1.;
    }
    double rx = round(px / lidu) * lidu;
    double ry = round(py / lidu) * lidu;
    return {rx, ry};
}

double TopoVec2::crossProduct(const TopoVec2& that)
{
    return this->px * that.py - this->py * that.px;
}

double TopoVec2::dotProduct(const TopoVec2& that) const
{
    return this->px * that.px + this->py * that.py;
}

bool TopoVec2::operator==(const TopoVec2& that) const
{
    return that.px == this->px && that.py == this->py;
}

bool TopoVec2::operator!=(const TopoVec2& that) const
{
    return !this->operator==(that);
}

double TopoVec2::tan() const
{
    return atan2(py, px) * 180 / M_PI;
}

