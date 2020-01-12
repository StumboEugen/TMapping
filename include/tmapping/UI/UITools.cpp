//
// Created by stumbo on 2019/12/27.
//

#include "UITools.h"
#include "tmapping/expDataTypes/ExpDataTypes.h"

#include <QPainter>

void tmap::UIT::drawGate(QPainter* painter, Gate* gate2draw, bool useGatePose, bool drawDetail)
{
    auto oriPen = painter->pen();
    auto offset = TopoVec2QPt(gate2draw->getPos());
    if (useGatePose) {
        painter->translate(offset);
    }

    auto halfNorVec = gate2draw->getNormalVec() / 2;
    if (gate2draw->type() == GateType::GateWay) {
        auto halfGatePos = UIT::TopoVec2QPt(
                halfNorVec.rotate(90).changeLen(DOOR_RAD / 2));

        /// 把路口下的边框用白色覆盖
        QPen whiteBasePen{Qt::white, 5};
        whiteBasePen.setCapStyle(Qt::RoundCap);
        painter->setPen(whiteBasePen);
        painter->drawLine(halfGatePos, -halfGatePos);
        painter->setPen(oriPen);

        /// 画出路口的虚线
        QPen dotPen{Qt::black, 3};
        dotPen.setStyle(Qt::DotLine);
        dotPen.setDashOffset(1);
        painter->setPen(dotPen);
        painter->drawLine(halfGatePos, -halfGatePos);
        painter->setPen(oriPen);

//        painter->setPen({Qt::black, 3});
//        painter->drawEllipse(QRectF{-2, -2, 4, 4});
    }
    if (gate2draw->type() == GateType::Door) {
        auto doorChainPos = UIT::TopoVec2QPt(
                halfNorVec.rotate(90).changeLen(DOOR_RAD / 2));

        Door* door = dynamic_cast<Door*>(gate2draw);

        /// 把门下的边框用白色覆盖
        QPen whiteBasePen{Qt::white, 5};
        whiteBasePen.setCapStyle(Qt::RoundCap);
        painter->setPen(whiteBasePen);
        painter->drawLine(doorChainPos, -doorChainPos);
        painter->setPen(oriPen);

        /// 绘制圆弧与虚线
        QPointF dia(UIT::QMeter(DOOR_RAD),UIT::QMeter(DOOR_RAD));
        QRectF rec{doorChainPos + dia, doorChainPos - dia};
        painter->setPen(QPen{Qt::black, 1, Qt::DotLine});
        painter->drawArc(rec,static_cast<int>(halfNorVec.tan() - 90) * 16,-16 * 90);
        painter->drawLine(doorChainPos,
                doorChainPos + UIT::TopoVec2QPt(-halfNorVec.changeLen(DOOR_RAD)));
        painter->drawLine(doorChainPos, -doorChainPos);
        painter->setPen(oriPen);

        /// 绘制门的主体
        QPen doorPen{Qt::black, 5};
        doorPen.setCapStyle(Qt::RoundCap);
        painter->setPen(doorPen);
        if (door->isOpened()) {
            painter->drawLine(doorChainPos,
                    doorChainPos + UIT::TopoVec2QPt(-halfNorVec.changeLen(DOOR_RAD)));
        } else {
            painter->drawLine(doorChainPos, -doorChainPos);
        }
        painter->setPen(oriPen);
    }

    if (useGatePose) {
        painter->translate(-offset);
    }
    painter->setPen(oriPen);
}

void tmap::UIT::drawLandMark(QPainter* painter, PosLandmark* lm2draw, bool usePos)
{
    auto oriPen = painter->pen();
    auto offset = TopoVec2QPt(lm2draw->getPos());
    if (usePos) {
        painter->translate(offset);
    }

    double r = UIT::QMeter(LANDMARK_RAD);
    painter->setBrush(Qt::green);
    painter->drawEllipse({QPointF{-r, -r}, QPointF{r, r}});

    if (usePos) {
        painter->translate(-offset);
    }
    painter->setPen(oriPen);
}

