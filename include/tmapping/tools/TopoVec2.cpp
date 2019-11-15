//
// Created by stumbo on 2019/11/15.
//

#include <iomanip>
#include <cmath>
#include <iostream>
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
    TopoVec2 vec2 = that;
    vec2 -= *this;
    return vec2;
}

double TopoVec2::len() const
{
    return sqrt(px * px + py * py);
}

TopoVec2 TopoVec2::unitVec() const
{
    if (px != 0.0 && py != 0.0) {
        TopoVec2 topoVec2 = *this;
        topoVec2 /= topoVec2.len();
        return topoVec2;
    } else {
        cerr << FILE_AND_LINE << " You are trying to get a unit vec from TopoVec2(0,0)!";
        return {};
    }
}

ostream& tmap::operator<<(ostream& os, const TopoVec2& pos)
{
    auto oldf = os.setf(ios::fixed);
    os << '(' << pos.px << ", " << pos.py << ')';
    os.setf(oldf);
    return os;
}

