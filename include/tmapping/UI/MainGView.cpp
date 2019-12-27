//
// Created by stumbo on 2019/12/20.
//

#include <iostream>

#include <QWheelEvent>

#include "MainGView.h"
#include "tmapping/MergedExp.h"
#include "tmapping/Exp.h"
#include "UITools.h"

using namespace std;

tmap::QNode::QNode(const ExpDataPtr& relatedExpData)
{
    auto exp = make_shared<Exp>(relatedExpData, 0);
    auto mergedExp = MergedExp::singleMergedFromExp(std::move(exp));
    relatedMergedExp = std::move(mergedExp);
}

tmap::QNode::~QNode() {
    if (scene()) {
        scene()->removeItem(this);
    }
}

QRectF tmap::QNode::boundingRect() const
{
    if (mBoundingRect.isEmpty()) {
        auto bRect = relatedMergedExp->getMergedExpData()->getOutBounding(0.5);
        mBoundingRect.setTopLeft(UIT::TopoVec2QPt({bRect[2], bRect[0]}));
        mBoundingRect.setBottomRight(UIT::TopoVec2QPt({bRect[3], bRect[1]}));
    }
    return mBoundingRect;
}

void
tmap::QNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    const auto& centerPoint = boundingRect().center();
    QPointF halfDia{UIT::QMeter(0.3), UIT::QMeter(0.3)};
    QRectF middleHalfSq{centerPoint + halfDia, centerPoint - halfDia};

    switch (relatedMergedExp->getMergedExpData()->type()) {

        case ExpDataType::Intersection:
            painter->setBrush(Qt::yellow);
            painter->drawEllipse(middleHalfSq);
            for (const auto& gate : relatedMergedExp->getMergedExpData()->getGates()) {
                UIT::drawGate(painter, gate.get(), true);
                painter->drawLine(centerPoint, UIT::TopoVec2QPt(gate->getPos()));
            }
            break;
        case ExpDataType::Corridor:
            //TODO
            break;
        case ExpDataType::Stair:
            cerr << FILE_AND_LINE << " unimplemented exp type!" << endl;
            break;
        case ExpDataType::BigRoom:
            cerr << FILE_AND_LINE << " unimplemented exp type!" << endl;
            break;
        case ExpDataType::SmallRoom:
            auto bRect = relatedMergedExp->getMergedExpData()->getOutBounding();
            auto tl = UIT::TopoVec2QPt({bRect[2], bRect[0]});
            auto br = UIT::TopoVec2QPt({bRect[3], bRect[1]});
            painter->drawRect(QRectF{tl,br});
            painter->setBrush(Qt::yellow);
            painter->drawRect(middleHalfSq);
            for (const auto& gate : relatedMergedExp->getMergedExpData()->getGates()) {
                UIT::drawGate(painter, gate.get(), true);
//                painter->drawLine(centerPoint, UIT::TopoVec2QPt(gate->getPos()));
            }
            break;
    }
}

tmap::MainGView::MainGView(QWidget* parent) : QGraphicsView(parent)
{
    setScene(&mScene4FakeMap);
}

static constexpr double SCALE_MAX = 10.0;
static constexpr double SCALE_TIME = 1.2;

void tmap::MainGView::wheelEvent(QWheelEvent* event)
{
    if (event->modifiers() & Qt::ControlModifier) {
        auto curTrans = transform();
        auto curScale = curTrans.m11();
        if (event->delta() > 0 && curScale <= SCALE_MAX) {
            curTrans.scale(SCALE_TIME, SCALE_TIME);
        }
        else if (event->delta() < 0 && curScale >= 1 / SCALE_MAX) {
            curTrans.scale(1 / SCALE_TIME, 1 / SCALE_TIME);
        }
        setMatrix(curTrans.toAffine());
    } else {
        QGraphicsView::wheelEvent(event);
    }
}

void tmap::MainGView::addNode2FakeMap(const tmap::ExpDataPtr& usedExpData)
{
    auto qNode = make_shared<QNode>(usedExpData->clone());
    mNodesInFakeMap.insert(qNode);
    mScene4FakeMap.addItem(qNode.get());
    qNode->setFlag(QGraphicsItem::ItemIsMovable);
}
