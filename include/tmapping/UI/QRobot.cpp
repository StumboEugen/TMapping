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

tmap::QRobot::QRobot(tmap::QNodePtr at, bool directMove)
        : currentAtNode(std::move(at)) ,
          currentExp(make_shared<Exp>(currentAtNode->expData()->clone(),
                  GATEID_BEGINNING_POINT)),
          atLM(SubNodeType::GATE, 0),
          directMove(directMove)
{
    currentAtNode->scene()->addItem(this);
    double r = UIT::QMeter(0.13);
    setRect({QPointF{-r, -r}, QPointF{r, r}});
    QColor c(Qt::red);
    c.setAlphaF(0.5);
    setBrush(c);
    updatePos();
}

tmap::QRobot::~QRobot()
{
    if (scene()) {
        scene()->removeItem(this);
    }
}

void QRobot::updatePos()
{
    if (!directMove) {
        switch (atLM.type) {
            case SubNodeType::LandMark:
                setPos(currentAtNode->plmQPos(atLM.index));
                break;
            case SubNodeType::GATE:
                setPos(currentAtNode->gateQPos(atLM.index));
                break;
            default:
                cerr << FILE_AND_LINE << " a UNKNOWN EXPDATA VERTEX TYPE:"
                     << static_cast<int32_t>(atLM.type)
                     << endl;
        }
    } else {
        setPos(currentAtNode->mapToScene(currentAtNode->boundingRect().center()));
    }
    update();
}

bool QRobot::try2move(QPointF scenePos)
{
    if (!directMove) {
        bool clickAtGate = false;
        auto newNode = atLM;
        auto nodePos = currentAtNode->mapFromScene(scenePos);
        const auto& expData = currentAtNode->expData();
        auto gid = expData->findGateAtPos(
                UIT::QPt2TopoVec(nodePos), UIT::pix2meter(15));
        if (gid >= 0) {
            clickAtGate = true;
            newNode.type = SubNodeType::GATE;
            newNode.index = gid;
        } else {
            auto PLMid = expData->findLmAtPos(
                    UIT::QPt2TopoVec(nodePos), UIT::pix2meter(15));
            if (PLMid >= 0) {
                newNode.type = SubNodeType::LandMark;
                newNode.index = PLMid;
            }
        }

        if (newNode != atLM) {
            currentExp->expData()->addSubLink(newNode.type, newNode.index,
                                              atLM.type, atLM.index, true);
            atLM = newNode;
            updatePos();
        }
        return clickAtGate;
    } else {
        auto nodePos = currentAtNode->mapFromScene(scenePos);
        const auto& expData = currentAtNode->expData();
        auto gid = expData->findGateAtPos(
                UIT::QPt2TopoVec(nodePos), UIT::pix2meter(15));
        if (gid >= 0) {
            atLM.type = SubNodeType::GATE;
            atLM.index = gid;
            return true;
        } else {
            return false;
        }
    }
}

ExpPtr QRobot::try2ThroughGate()
{
    if (atLM.type == SubNodeType::GATE) {
        if (auto linkedQNode = currentAtNode->qNodeAt(atLM.index)) {
            currentExp->setLeftGate(atLM.index);
            atLM.type = SubNodeType::GATE;
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

ExpPtr QRobot::moveThroughGate(GateID gateId)
{
    atLM.type = SubNodeType::GATE;
    atLM.index = gateId;
    return try2ThroughGate();
}

GateID QRobot::enterGate() const
{
    return currentExp->getEnterGate();
}
