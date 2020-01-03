//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_TOPOVEC2_H
#define TMAPPING_TOPOVEC2_H

#include <ostream>
#include <json/json.h>
#include "TopoParams.h"

namespace tmap
{

struct TopoVec2
{
    double px;
    double py;

    TopoVec2() = default;
    TopoVec2(double px, double py);
    explicit TopoVec2(const Jsobj& jvec);

    friend std::ostream& operator<<(std::ostream& os, const TopoVec2& pos);

    TopoVec2& operator*= (double d);

    TopoVec2 operator* (double d) const;

    TopoVec2& operator/= (double d);

    TopoVec2 operator/ (double d) const;

    TopoVec2& operator+= (const TopoVec2& that);

    TopoVec2 operator+ (const TopoVec2& that) const;

    TopoVec2& operator-= (const TopoVec2& that);

    TopoVec2 operator- (const TopoVec2& that) const;

    TopoVec2 operator- () const;

    bool operator==(const TopoVec2& that) const;

    bool operator!=(const TopoVec2& that) const;

    double len() const;

    Json::Value toJS() const;

    /**
     * @brief 得到单位化长度的Vec, 但是如果原来为(0,0), 会得到(1,0)
     */
    TopoVec2 unitize() const;

    const TopoVec2& restrictDir(size_t nSlice = 8);

    /**
     * @brief 产生一个新的Vec, 但是长度为 l
     */
    TopoVec2 changeLen(double l) const;

    /// ENU, 逆时针为正
    TopoVec2 rotate(int degree) const;

    /**
     * @brief 取整到lidu
     */
    TopoVec2 round2(double lidu = 1.0) const;

    double crossProduct(const TopoVec2& that);

    double dotProduct(const TopoVec2& that);
};

std::ostream& operator<<(std::ostream& os, const TopoVec2& pos);

}


#endif //TMAPPING_TOPOVEC2_H
