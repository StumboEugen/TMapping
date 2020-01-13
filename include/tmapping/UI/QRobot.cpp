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
        : currentAtNode(std::move(at)) ,
          currentExp(make_shared<Exp>(currentAtNode->expData()->clone(), GATEID_BEGINNING_POINT))
{
    currentAtNode->scene()->addItem(this);
    double r = UIT::QMeter(0.13);
    setRect({QPointF{-r, -r}, QPointF{r, r}});
    QColor c(Qt::red);
    c.setAlphaF(0.5);
    setBrush(c);
    setPos(currentAtNode->gateQPos(0, true));
    atLM.type = LMTYPE_GATE;
    atLM.index = 0;
}

tmap::QRobot::~QRobot()
{
    if (scene()) {
        scene()->removeItem(this);
    }
}

void QRobot::updatePos()
{
    switch (atLM.type) {
        case LMTYPE_LM:
            setPos(currentAtNode->plmQPos(atLM.index));
            break;
        case LMTYPE_GATE:
            setPos(currentAtNode->gateQPos(atLM.index));
            break;
        default:
            cerr << FILE_AND_LINE << " a UNKNOWN EXPDATA VERTEX TYPE:" << atLM.type << endl;
    }
    update();
}

bool QRobot::try2move(QPointF scenePos)
{
    bool clickAtGate = false;
    ExpData::Vertex newVertex = atLM;
    auto nodePos = currentAtNode->mapFromScene(scenePos);
    const auto& expData = currentAtNode->expData();
    auto gid = expData->findGateAtPos(
            UIT::QPt2TopoVec(nodePos), UIT::pix2meter(15));
    if (gid >= 0) {
        clickAtGate = true;
        newVertex.type = LMTYPE_GATE;
        newVertex.index = gid;
    } else {
        auto PLMid = expData->findLmAtPos(
                UIT::QPt2TopoVec(nodePos), UIT::pix2meter(15));
        if (PLMid >= 0) {
            newVertex.type = LMTYPE_LM;
            newVertex.index = PLMid;
        }
    }
    
    if (newVertex != atLM) {
        currentExp->expData()->addSubLink(newVertex.type, newVertex.index,
                            atLM.type, atLM.index, true);
        atLM = newVertex;
        updatePos();
    }
    return clickAtGate;
}

ExpPtr QRobot::try2ThroughGate()
{
    if (atLM.type == LMTYPE_GATE) {
        if (auto linkedQNode = currentAtNode->qNodeAt(atLM.index)) {
            currentExp->setLeftGate(atLM.index);
            atLM.type = LMTYPE_GATE;
            atLM.index = currentAtNode->linkedGIDAt(atLM.index);
            ExpPtr newExp = make_shared<Exp>(
                    linkedQNode->expData()->clone(), atLM.index);
            ExpPtr oldExp = currentExp;
            currentExp = newExp;
            currentAtNode = linkedQNode;
            updatePos();
            return oldExp;
        }
    }
    return nullptr;
}

const QNodePtr& QRobot::atNode() const
{
    return currentAtNode;
}
