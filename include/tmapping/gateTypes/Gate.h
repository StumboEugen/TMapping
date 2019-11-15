//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_GATE_H
#define TMAPPING_GATE_H

#include "tmapping/tools/TopoVec2.h"

namespace tmap
{

enum class GateType{GateWay, Door};

class Gate
{
    TopoVec2 pos;
    TopoVec2 normalVec;
    double possibility = 1.0;

public:
    Gate(const TopoVec2& pos, const TopoVec2& normalVec);

    virtual GateType type() = 0;

    const TopoVec2& getPos() const;

    const TopoVec2& getNormalVec() const;

    double getPossibility() const;

    void setPossibility(double possibility);
};

}


#endif //TMAPPING_GATE_H
