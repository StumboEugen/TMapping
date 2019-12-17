//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_TOPOVEC2_H
#define TMAPPING_TOPOVEC2_H

#include <ostream>
#include <json/json.h>

namespace tmap
{

struct TopoVec2
{
    double px;
    double py;

    TopoVec2() = default;
    TopoVec2(double px, double py);

    friend std::ostream& operator<<(std::ostream& os, const TopoVec2& pos);

    TopoVec2& operator*= (double d);

    TopoVec2 operator* (double d) const;

    TopoVec2& operator/= (double d);

    TopoVec2 operator/ (double d) const;

    TopoVec2& operator+= (const TopoVec2& that);

    TopoVec2 operator+ (const TopoVec2& that) const;

    TopoVec2& operator-= (const TopoVec2& that);

    TopoVec2 operator- (const TopoVec2& that) const;

    double len() const;

    TopoVec2 unitVec() const;

    Json::Value toJS() const;
};

std::ostream& operator<<(std::ostream& os, const TopoVec2& pos);

}


#endif //TMAPPING_TOPOVEC2_H
