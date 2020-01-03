//
// Created by stumbo on 2019/12/23.
//

#ifndef TMAPPING_INCLUDE_TMAPPING_UI_UITOOLS_H
#define TMAPPING_INCLUDE_TMAPPING_UI_UITOOLS_H

#include "tmapping/tools/TopoVec2.h"

#include <QPointF>

class QPainter;

namespace tmap {

class Gate;

static constexpr double METER2PIX = 60.0;
static constexpr double DOOR_RAD = 0.4;

/// UI TOOLS
namespace UIT {
inline TopoVec2 QPt2TopoVec(const QPointF& qPoint)
{
    return {qPoint.x() / METER2PIX, qPoint.y() / -METER2PIX};
}

inline QPointF TopoVec2QPt(const TopoVec2& topoVec)
{
    return {topoVec.px * METER2PIX, topoVec.py * -METER2PIX};
}

inline double QMeter(double meter)
{
    return METER2PIX * meter;
}

inline double pix2meter(double pix)
{
    return pix / METER2PIX;
}

void drawGate(
        QPainter* painter,
        Gate* gate2draw,
        bool useGatePose = false,
        bool drawDetail =true);

} /// namespace UIT

}

#endif //TMAPPING_INCLUDE_TMAPPING_UI_UITOOLS_H
