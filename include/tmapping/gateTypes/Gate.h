//
// Created by stumbo on 2019/11/15.
//

#ifndef TMAPPING_GATE_H
#define TMAPPING_GATE_H

#include "tmapping/tools/TopoVec2.h"
#include <memory>
#include "../tools/TopoTools.h"

namespace tmap
{
class Gate;

using GatePtr = std::shared_ptr<Gate>;
using GateWePtr = std::weak_ptr<Gate>;

// open close 的设置是为了UI界面的方便
enum class GateType{GateWay, Door, DoorOpened, DoorClosed};

class Gate
{
    TopoVec2 mPos;
    TopoVec2 mNormalVec;
    double possibility = 1.0;

protected:
    void copy2(Gate* target);

    void mergeBasicInfo(const Gate* A, const Gate* B
            , const TopoVec2& BPos , double weightA);

public:
    Gate(const TopoVec2& pos, const TopoVec2& normalVec);

    static GatePtr madeFromJS(const Jsobj& jgate);

    virtual GateType type() const = 0 ;

    const TopoVec2& getPos() const;

    const TopoVec2& getNormalVec() const;

    double getPossibility() const;

    void setPossibility(double poss);

    virtual Json::Value toJS() const;

    static std::string typeStr(GateType type);

    virtual GatePtr clone() = 0;

    virtual GatePtr
    newMergedGate(const GatePtr& that, const TopoVec2& thatPos, double thisWeight) const = 0;

    void setPos(const TopoVec2& pos);

    void setNormalVec(const TopoVec2& normalVec);

    virtual bool alike(const GatePtr& that) const;
};

}


#endif //TMAPPING_GATE_H
