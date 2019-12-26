//
// Created by stumbo on 2019/12/20.
//

#include "MainGView.h"

#include <QWheelEvent>


tmap::QNode::~QNode() {
    if (scene()) {
        scene()->removeItem(this);
    }
}

QRectF tmap::QNode::boundingRect() const
{
    return QRectF();
}

void
tmap::QNode::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{

}

tmap::MainGView::MainGView(QWidget* parent) : QGraphicsView(parent)
{
    setScene(&mScene4FakeMap);
    mScene4FakeMap.addLine(0, 0, 100, 100);
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
