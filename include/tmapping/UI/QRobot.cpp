//
// Created by stumbo on 2020/1/9.
//

#include <QBrush>
#include <QGraphicsScene>
#include "QRobot.h"
#include "UITools.h"

#include <iostream>

using namespace std;
using namespace tmap;

tmap::QRobot::QRobot(tmap::QNodePtr at)
        : atNode(std::move(at))
{
    atNode->scene()->addItem(this);
    double r = UIT::QMeter(0.13);
    setRect({QPointF{-r, -r}, QPointF{r, r}});
    QColor c(Qt::red);
    c.setAlphaF(0.5);
    setBrush(c);
    auto cent = atNode->boundingRect().center();
    setPos(atNode->mapToScene(cent));
}

tmap::QRobot::~QRobot()
{
    if (scene()) {
        scene()->removeItem(this);
    }
}

void QRobot::updatePos()
{
    auto cent = atNode->boundingRect().center();
    setPos(atNode->mapToScene(cent));
    update();
}

ExpPtr QRobot::try2move(QPointF scenePos)
{
    auto nodePos = atNode->mapFromScene(scenePos);
    auto gid = atNode->expData()->findGateAtPos(
            UIT::QPt2TopoVec(nodePos), UIT::pix2meter(15));
    if (gid >= 0) {
        if (auto toQNode = atNode->qNodeAt(gid)) {
            theLastMovedExp = make_shared<Exp>(toQNode->expData(), atNode->linkedGIDAt(gid));
            atNode = toQNode;
            updatePos();
            return theLastMovedExp;
        }
    }
    return nullptr;
}