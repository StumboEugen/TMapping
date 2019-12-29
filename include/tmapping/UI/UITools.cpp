//
// Created by stumbo on 2019/12/27.
//

#include "UITools.h"
#include "tmapping/expDataTypes/ExpDataTypes.h"

#include <QPainter>

void tmap::UIT::drawGate(QPainter* painter, Gate* gate2draw, bool useGatePose)
{
    auto oldPen = painter->pen();
    auto offset = TopoVec2QPt(gate2draw->getPos());
    if (useGatePose) {
        painter->translate(offset);
    }

    auto halfNorVec = gate2draw->getNormalVec() / 2;
    auto p2 = UIT::TopoVec2QPt(halfNorVec);
    if (gate2draw->type() == GateType::GateWay) {
        painter->setPen({Qt::black, 3});
        painter->drawLine({0, 0}, p2);
        painter->drawEllipse(QRectF{-2, -2, 4, 4});
    }
    if (gate2draw->type() == GateType::Door) {
        painter->drawLine({0, 0}, p2);
        auto halfDoor = halfNorVec.rotate(90).changeLen(0.2);
        Door* door = dynamic_cast<Door*>(gate2draw);
        QPen pen{Qt::black, 2, door->isOpened() ? Qt::DotLine : Qt::SolidLine};
        pen.setDashOffset(2);
        painter->setPen(pen);
        painter->drawLine(UIT::TopoVec2QPt(halfDoor), UIT::TopoVec2QPt(-halfDoor));
        painter->drawEllipse(QRectF{-2, -2, 4, 4});
    }

    if (useGatePose) {
        painter->translate(-offset);
    }
    painter->setPen(oldPen);
}

